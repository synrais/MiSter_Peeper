// mister_peeper.cpp — hash-timed unchanged, stable 100 Hz, linear avg, HSV color naming
// Prints: time=HH:MM:SS  unchanged=secs  avg_rgb=#RRGGBB  color=Name

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <csignal>
#include <cmath>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <algorithm>

static constexpr int    kStep      = 16;   // sparse sampling grid
static constexpr size_t FB_BASE_ADDRESS = 0x20000000u;
static constexpr size_t MAP_LEN         = 2048u * 1024u * 12u; // ~24 MiB
static constexpr long   kHz100_ns       = 10'000'000L;         // 10 ms tick

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
static inline void sleep_until_ns(uint64_t t_ns){
    struct timespec ts{ (time_t)(t_ns/1000000000ull), (long)(t_ns%1000000000ull) };
    // Use absolute-time sleep to avoid drift
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

static inline uint32_t hash_rgb(uint32_t h,uint8_t r,uint8_t g,uint8_t b){
    h^=((uint32_t)r<<16)^((uint32_t)g<<8)^(uint32_t)b;
    h^=h<<13; h^=h>>17; h^=h<<5; return h;
}

static inline void fmt_hms(double s,char* out,size_t n){
    int S=(int)s; int H=S/3600; int M=(S%3600)/60; S%=60;
    snprintf(out,n,"%02d:%02d:%02d",H,M,S);
}

// --------- sRGB <-> Linear (LUT) for accurate averaging ----------
static uint32_t g_lut_lin[256]; // linear * 2^20 (fixed-point)
static inline void init_lut(){
    for(int i=0;i<256;i++){
        double s=i/255.0;
        double lin = (s<=0.04045)? (s/12.92) : pow((s+0.055)/1.055,2.4);
        g_lut_lin[i]=(uint32_t)llround(lin*(double)(1u<<20));
    }
}
static inline uint8_t lin_to_srgb(double lin){
    if(lin<=0.0) return 0; if(lin>=1.0) return 255;
    double s = (lin<=0.0031308)? 12.92*lin : 1.055*pow(lin,1.0/2.4)-0.055;
    int v=(int)llround(s*255.0); if(v<0) v=0; if(v>255) v=255; return (uint8_t)v;
}

// --------- HSV-based human-readable color naming ----------
static inline const char* name_from_hsv(uint8_t R,uint8_t G,uint8_t B){
    int maxv = std::max({(int)R,(int)G,(int)B});
    int minv = std::min({(int)R,(int)G,(int)B});
    int delta = maxv - minv;

    double V = maxv / 255.0;
    double S = (maxv == 0) ? 0.0 : (double)delta / (double)maxv;

    // Brightness/saturation gates to avoid mislabeling vivid colors as "Black"
    if (V < 0.08) return "Black";
    if (S < 0.10) {
        if (V > 0.92) return "White";
        if (V > 0.55) return "Silver";
        return "Gray";
    }

    double H = 0.0;
    if (delta != 0) {
        if (maxv == R)      H = 60.0 * fmod(((double)(G - B) / delta), 6.0);
        else if (maxv == G) H = 60.0 * (((double)(B - R) / delta) + 2.0);
        else                H = 60.0 * (((double)(R - G) / delta) + 4.0);
        if (H < 0.0) H += 360.0;
    }

    if (H < 15 || H >= 345) return (V > 0.6 ? "Red" : "Dark Red");
    if (H < 45)   return "Orange";
    if (H < 65)   return "Yellow";
    if (H < 90)   return "Chartreuse";
    if (H < 150)  return "Green";
    if (H < 200)  return "Cyan";
    if (H < 230)  return "Turquoise";
    if (H < 255)  return "Blue";
    if (H < 285)  return "Violet";
    if (H < 330)  return "Magenta";
    return "Rose";
}

int main(){
    signal(SIGINT,on_sig); signal(SIGTERM,on_sig);
    init_lut();

    // Map scaler
    int fd=open("/dev/mem",O_RDONLY|O_SYNC);
    if(fd<0){ perror("open(/dev/mem)"); return 1; }
    void* map=mmap(nullptr,MAP_LEN,PROT_READ,MAP_SHARED,fd,FB_BASE_ADDRESS);
    if(map==MAP_FAILED){ perror("mmap"); close(fd); return 1; }
    volatile uint8_t* base=(volatile uint8_t*)map;

    // Read header
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

    // Pixel loaders (RGB order; swap to BGR here if your source is BGR)
    auto load_rgb24=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgba32=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgb565=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){
        uint16_t v=(uint16_t)p[0]|((uint16_t)p[1]<<8);
        r=(uint8_t)(((v>>11)&0x1F)*255/31);
        g=(uint8_t)(((v>>5)&0x3F)*255/63);
        b=(uint8_t)(( v     &0x1F)*255/31);
    };

    // Change detection state
    uint32_t last_hash=0; bool first=true;
    uint64_t start_ns=now_ns(), last_change_ns=start_ns;

    // Previous per-buffer counters
    uint8_t prev_fc[3] = {
        attr_ptrs[0][0],
        static_cast<uint8_t>(triple ? attr_ptrs[1][0] : 0),
        static_cast<uint8_t>(triple ? attr_ptrs[2][0] : 0)
    };

    // Cadence anchor
    uint64_t next_tick = now_ns(); // start immediately

    while(g_run){
        // Stable ~100 Hz
        next_tick += kHz100_ns;
        sleep_until_ns(next_tick);
        if(!g_run) break;

        // Touch frame counters once per tick
        (void)sum_fc(); // optional read; we don't busy-wait

        // Pick active buffer (which counter ticked)
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
        uint64_t rlin=0, glin=0, blin=0, n=0;
        uint32_t hsh=2166136261u;

        for(unsigned y=0;y<height;y+=ys){
            const volatile uint8_t* row = pix + (size_t)y*line;
            __builtin_prefetch((const void*)(pix + (size_t)(y+ys)*line), 0, 1);
            if(fmt==RGB24){
                for(unsigned x=0;x<width;x+=xs){
                    const volatile uint8_t* p=row + (size_t)x*3;
                    uint8_t r,g,b; load_rgb24(p,r,g,b);
                    rlin += g_lut_lin[r]; glin += g_lut_lin[g]; blin += g_lut_lin[b];
                    hsh=hash_rgb(hsh,r,g,b); n++;
                }
            } else if(fmt==RGBA32){
                for(unsigned x=0;x<width;x+=xs){
                    const volatile uint8_t* p=row + (size_t)x*4;
                    uint8_t r,g,b; load_rgba32(p,r,g,b);
                    rlin += g_lut_lin[r]; glin += g_lut_lin[g]; blin += g_lut_lin[b];
                    hsh=hash_rgb(hsh,r,g,b); n++;
                }
            } else { // RGB565
                for(unsigned x=0;x<width;x+=xs){
                    const volatile uint8_t* p=row + (size_t)x*2;
                    uint8_t r,g,b; load_rgb565(p,r,g,b);
                    rlin += g_lut_lin[r]; glin += g_lut_lin[g]; blin += g_lut_lin[b];
                    hsh=hash_rgb(hsh,r,g,b); n++;
                }
            }
        }

        // Change detection: hash only
        uint64_t t_ns=now_ns();
        if(first){ last_hash=hsh; last_change_ns=t_ns; first=false; }
        else if(hsh!=last_hash){ last_hash=hsh; last_change_ns=t_ns; }

        // Compute avg in linear, convert to sRGB
        double inv = n? 1.0/(double)n : 0.0;
        double r_lin = (double)rlin * inv / (double)(1u<<20);
        double g_lin = (double)glin * inv / (double)(1u<<20);
        double b_lin = (double)blin * inv / (double)(1u<<20);

        uint8_t R = lin_to_srgb(r_lin);
        uint8_t G = lin_to_srgb(g_lin);
        uint8_t B = lin_to_srgb(b_lin);
        const char* cname = name_from_hsv(R,G,B);

        // Output
        double unchanged_s=(t_ns-last_change_ns)/1e9;
        double elapsed_s  =(t_ns-start_ns)/1e9;
        char tbuf[16]; fmt_hms(elapsed_s,tbuf,sizeof(tbuf));

        printf("time=%s  unchanged=%.3f  avg_rgb=#%02X%02X%02X  color=%s\n",
               tbuf, unchanged_s, (unsigned)R,(unsigned)G,(unsigned)B, cname);
        fflush(stdout);
    }

    munmap((void*)base,MAP_LEN); close(fd); return 0;
}
