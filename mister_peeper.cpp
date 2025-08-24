// mister_peeper.cpp — per-frame, buffer-aware, hash-based unchanged timer, minimal output
// Prints: time=HH:MM:SS  unchanged=secs  avg_rgb=#RRGGBB  color=Name
// Set SHOW_HASH 1 to also print hash and buffer index for verification.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <csignal>
#include <cmath>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>

#define SHOW_HASH 0  // set to 1 for debug: shows hsh=0x???????? buf=N

static constexpr int    kStep      = 16;   // sparse grid (fast & sufficient)
static constexpr size_t FB_BASE_ADDRESS = 0x20000000u;
static constexpr size_t MAP_LEN         = 2048u * 1024u * 12u; // ~24 MiB

enum ScalerPixelFormat : uint8_t { RGB16 = 0, RGB24 = 1, RGBA32 = 2 };

#pragma pack(push,1)
struct FbHeader {
    uint8_t  ty;                // 0x01
    uint8_t  pixel_fmt;         // 0,1,2
    uint16_t header_len_be;
    uint16_t attributes_be;     // bit4 triple; bits7..5 frame counter
    uint16_t width_be;
    uint16_t height_be;
    uint16_t line_be;           // stride
    uint16_t out_width_be;
    uint16_t out_height_be;
};
#pragma pack(pop)

static inline uint16_t be16(uint16_t x){ return (uint16_t)((x>>8)|(x<<8)); }
static inline bool triple_buffered(uint16_t attr_be){ return (be16(attr_be) & (1u<<4)) != 0; }
static inline size_t fb_off(bool large, uint8_t idx){
    if(idx==0) return 0;
    return large ? (idx==1 ? 0x00800000u : 0x01000000u)
                 : (idx==1 ? 0x00200000u : 0x00400000u);
}

static volatile bool g_run=true;
static void on_sig(int){ g_run=false; }

static inline uint64_t now_ns(){
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
    return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec;
}
static inline uint32_t hash_rgb(uint32_t h,uint8_t r,uint8_t g,uint8_t b){
    // Same rolling mixer as your “accurate” version
    h^=((uint32_t)r<<16)^((uint32_t)g<<8)^(uint32_t)b;
    h^=h<<13; h^=h>>17; h^=h<<5; return h;
}
static inline void fmt_hms(double s,char* out,size_t n){
    int S=(int)s; int H=S/3600; int M=(S%3600)/60; S%=60;
    snprintf(out,n,"%02d:%02d:%02d",H,M,S);
}

// Small nearest-color palette (cheap & stable)
struct NamedColor { const char* name; uint8_t r,g,b; };
static const NamedColor kNamed[] = {
    {"Black",0,0,0}, {"White",255,255,255}, {"Silver",192,192,192}, {"Gray",128,128,128},
    {"Red",255,0,0}, {"Maroon",128,0,0}, {"Orange",255,165,0}, {"Brown",165,42,42},
    {"Yellow",255,255,0}, {"Olive",128,128,0}, {"Lime",0,255,0}, {"Green",0,128,0},
    {"Cyan",0,255,255}, {"Teal",0,128,128}, {"Blue",0,0,255}, {"Navy",0,0,128},
    {"Magenta",255,0,255}, {"Purple",128,0,128}, {"Pink",255,192,203}
};
static inline const char* name_for_rgb(unsigned R, unsigned G, unsigned B){
    const char* best="Unknown";
    unsigned best_d=~0u;
    for(const auto& c: kNamed){
        int dr=(int)R-(int)c.r, dg=(int)G-(int)c.g, db=(int)B-(int)c.b;
        unsigned d=(unsigned)(dr*dr + dg*dg + db*db);
        if(d<best_d){ best_d=d; best=c.name; }
    }
    return best;
}

int main(){
    signal(SIGINT,on_sig); signal(SIGTERM,on_sig);

    // Map scaler
    int fd=open("/dev/mem",O_RDONLY|O_SYNC);
    if(fd<0){ perror("open(/dev/mem)"); return 1; }
    void* map=mmap(nullptr,MAP_LEN,PROT_READ,MAP_SHARED,fd,FB_BASE_ADDRESS);
    if(map==MAP_FAILED){ perror("mmap"); close(fd); return 1; }
    volatile uint8_t* base=(volatile uint8_t*)map;

    // Read header once for geometry/format
    FbHeader h{}; memcpy(&h,(const void*)base,sizeof(FbHeader));
    if(h.ty!=0x01){
        fprintf(stderr,"error=header_not_found ty=%u\n",(unsigned)h.ty);
        munmap((void*)base,MAP_LEN); close(fd); return 3;
    }
    const uint16_t header_len=be16(h.header_len_be);
    const uint16_t width     =be16(h.width_be);
    const uint16_t height    =be16(h.height_be);
    const uint16_t line      =be16(h.line_be);
    const ScalerPixelFormat fmt=(ScalerPixelFormat)h.pixel_fmt;
    const bool triple = triple_buffered(h.attributes_be);

    // Detect small vs large triple by probing for valid headers
    auto hdr_ok=[&](size_t off){
        if(off+sizeof(FbHeader)>MAP_LEN) return false;
        FbHeader t{}; memcpy(&t,(const void*)(base+off),sizeof(FbHeader));
        return t.ty==0x01 && t.pixel_fmt<=2;
    };
    bool large = triple && !hdr_ok(0x00200000u) && hdr_ok(0x00800000u);

    // Attribute pointers (frame counters at header + 5)
    volatile const uint8_t* attr_ptrs[3] = {
        base + 5,
        base + fb_off(large,1) + 5,
        base + fb_off(large,2) + 5
    };

    // Helpers
    auto sum_fc=[&](){
        uint16_t s = attr_ptrs[0][0];
        if(triple){ s = (uint16_t)(s + attr_ptrs[1][0] + attr_ptrs[2][0]); }
        return s;
    };
    auto choose_active_idx = [&](const uint8_t prev[3], const uint8_t curr[3])->int{
        if(!triple) return 0;
        for(int i=0;i<3;i++) if(curr[i]!=prev[i]) return i; // the one that changed
        int idx=0; if(curr[1]>=curr[idx]) idx=1; if(curr[2]>=curr[idx]) idx=2; return idx; // fallback
    };

    // Pixel loaders (RGB order; if your source were BGR, swap here)
    auto load_rgb24=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgba32=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgb565=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){
        uint16_t v=(uint16_t)p[0]|((uint16_t)p[1]<<8);
        r=(uint8_t)(((v>>11)&0x1F)*255/31);
        g=(uint8_t)(((v>>5)&0x3F)*255/63);
        b=(uint8_t)(( v     &0x1F)*255/31);
    };

    // Change detection state (hash-only)
    uint32_t last_hash=0; bool first=true;
    uint64_t start_ns=now_ns(), last_change_ns=start_ns;

    // Previous per-buffer counters
    uint8_t prev_fc[3] = {
        attr_ptrs[0][0],
        static_cast<uint8_t>(triple ? attr_ptrs[1][0] : 0),
        static_cast<uint8_t>(triple ? attr_ptrs[2][0] : 0)
    };

    while(g_run){
        // Wait for next frame; poll gently at ~100 Hz (10 ms)
        uint16_t s0 = sum_fc();
        while(g_run && sum_fc()==s0) usleep(10000);
        if(!g_run) break;

        // Pick active buffer (the one that ticked)
        uint8_t curr_fc[3] = {
            attr_ptrs[0][0],
            static_cast<uint8_t>(triple ? attr_ptrs[1][0] : 0),
            static_cast<uint8_t>(triple ? attr_ptrs[2][0] : 0)
        };
        int buf = choose_active_idx(prev_fc, curr_fc);
        prev_fc[0]=curr_fc[0]; prev_fc[1]=curr_fc[1]; prev_fc[2]=curr_fc[2];

        // Sample that buffer once (no retries)
        const volatile uint8_t* pix = base + fb_off(large,(uint8_t)buf) + header_len;

        const int xs=kStep, ys=kStep;
        uint64_t rs=0,gs=0,bs=0,n=0;
        uint32_t hsh=2166136261u; // seed

        if(fmt==RGB24){
            for(unsigned y=0;y<height;y+=ys){
                const volatile uint8_t* row=pix + (size_t)y*line;
                for(unsigned x=0;x<width;x+=xs){
                    const volatile uint8_t* p=row + (size_t)x*3;
                    uint8_t r,g,b; load_rgb24(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        } else if(fmt==RGBA32){
            for(unsigned y=0;y<height;y+=ys){
                const volatile uint8_t* row=pix + (size_t)y*line;
                for(unsigned x=0;x<width;x+=xs){
                    const volatile uint8_t* p=row + (size_t)x*4;
                    uint8_t r,g,b; load_rgba32(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        } else { // RGB16
            for(unsigned y=0;y<height;y+=ys){
                const volatile uint8_t* row=pix + (size_t)y*line;
                for(unsigned x=0;x<width;x+=xs){
                    const volatile uint8_t* p=row + (size_t)x*2;
                    uint8_t r,g,b; load_rgb565(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        }

        // Change detection (hash-only)
        uint64_t t_ns=now_ns();
        if(first){
            last_hash=hsh; last_change_ns=t_ns; first=false;
        } else if(hsh!=last_hash){
            last_hash=hsh;
            last_change_ns=t_ns;
        }

        // Averages and output
        unsigned R=(unsigned)(n? (double)rs/n : 0.0);
        unsigned G=(unsigned)(n? (double)gs/n : 0.0);
        unsigned B=(unsigned)(n? (double)bs/n : 0.0);

        double unchanged_s=(t_ns-last_change_ns)/1e9;
        double elapsed_s  =(t_ns-start_ns)/1e9;
        char tbuf[16]; fmt_hms(elapsed_s,tbuf,sizeof(tbuf));

    #if SHOW_HASH
        printf("time=%s  unchanged=%.3f  avg_rgb=#%02X%02X%02X  color=%s  hsh=0x%08X  buf=%d\n",
               tbuf, unchanged_s, R,G,B, name_for_rgb(R,G,B), hsh, buf);
    #else
        printf("time=%s  unchanged=%.3f  avg_rgb=#%02X%02X%02X  color=%s\n",
               tbuf, unchanged_s, R,G,B, name_for_rgb(R,G,B));
    #endif
        fflush(stdout);
    }

    munmap((void*)base,MAP_LEN); close(fd); return 0;
}
