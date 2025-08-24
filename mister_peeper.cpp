// mister_peeper.cpp — per-frame, buffer-aware, minimal output + inline FPS + signature
// Each frame prints:
//   time=HH:MM:SS  unchanged=secs  fps=XX.X  sig=0xXXXXXXXX  avg_rgb=#RRGGBB  center_rgb=#RRGGBB

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

static constexpr int    kStep      = 16;   // sparse grid
static constexpr double kTolerance = 3.0;  // L1 delta on avg RGB to count as "changed"
static constexpr size_t FB_BASE_ADDRESS = 0x20000000u;
static constexpr size_t MAP_LEN         = 2048u * 1024u * 12u;

enum ScalerPixelFormat : uint8_t { RGB16 = 0, RGB24 = 1, RGBA32 = 2 };

#pragma pack(push,1)
struct FbHeader {
    uint8_t  ty;
    uint8_t  pixel_fmt;
    uint16_t header_len_be;
    uint16_t attributes_be;
    uint16_t width_be;
    uint16_t height_be;
    uint16_t line_be;
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

static inline uint64_t now_ns(){ struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts); return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec; }
static inline void nsleep(long ns){ struct timespec rq{0,ns}; nanosleep(&rq,nullptr); }
static inline void nanosnooze(){ nsleep(1000000L); }         // 1 ms
static inline void short_relax(){ asm volatile("" ::: "memory"); } // compiler barrier

static inline uint32_t hash_rgb(uint32_t h,uint8_t r,uint8_t g,uint8_t b){
    h^=((uint32_t)r<<16)^((uint32_t)g<<8)^(uint32_t)b; h^=h<<13; h^=h>>17; h^=h<<5; return h;
}
static inline void fmt_hms(double s,char* out,size_t n){ int S=(int)s; int H=S/3600; int M=(S%3600)/60; S%=60; snprintf(out,n,"%02d:%02d:%02d",H,M,S); }

int main(){
    signal(SIGINT,on_sig); signal(SIGTERM,on_sig);

    int fd=open("/dev/mem",O_RDONLY|O_SYNC);
    if(fd<0){ perror("open(/dev/mem)"); return 1; }
    void* map=mmap(nullptr,MAP_LEN,PROT_READ,MAP_SHARED,fd,FB_BASE_ADDRESS);
    if(map==MAP_FAILED){ perror("mmap"); close(fd); return 1; }
    volatile uint8_t* base=(volatile uint8_t*)map;

    FbHeader h{}; memcpy(&h,(const void*)base,sizeof(FbHeader));
    if(h.ty!=0x01){ fprintf(stderr,"error=header_not_found\n"); munmap((void*)base,MAP_LEN); close(fd); return 3; }

    const uint16_t header_len=be16(h.header_len_be);
    const uint16_t width     =be16(h.width_be);
    const uint16_t height    =be16(h.height_be);
    const uint16_t line      =be16(h.line_be);
    const ScalerPixelFormat fmt=(ScalerPixelFormat)h.pixel_fmt;
    const bool triple = triple_buffered(h.attributes_be);

    auto hdr_ok=[&](size_t off){
        if(off+sizeof(FbHeader)>MAP_LEN) return false;
        FbHeader t{}; memcpy(&t,(const void*)(base+off),sizeof(FbHeader));
        return t.ty==0x01 && t.pixel_fmt<=2;
    };
    bool large = triple && !hdr_ok(0x00200000u) && hdr_ok(0x00800000u);

    // Attribute pointers (frame counters at header+5)
    volatile const uint8_t* attr_ptrs[3] = {
        base + 5,
        base + fb_off(large,1) + 5,
        base + fb_off(large,2) + 5
    };

    auto sum_fc=[&](){
        uint16_t s = attr_ptrs[0][0];
        if(triple){ s = (uint16_t)(s + attr_ptrs[1][0] + attr_ptrs[2][0]); }
        return s;
    };
    auto choose_active_idx = [&](const uint8_t prev[3], const uint8_t curr[3])->int{
        if(!triple) return 0;
        for(int i=0;i<3;i++) if(curr[i]!=prev[i]) return i;
        int idx=0; if(curr[1]>=curr[idx]) idx=1; if(curr[2]>=curr[idx]) idx=2; return idx;
    };

    auto load_rgb24=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgba32=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgb565=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ uint16_t v=(uint16_t)p[0]|((uint16_t)p[1]<<8); r=(uint8_t)(((v>>11)&0x1F)*255/31); g=(uint8_t)(((v>>5)&0x3F)*255/63); b=(uint8_t)((v&0x1F)*255/31); };

    uint32_t last_hash=0; bool first=true;
    uint64_t start_ns=now_ns(), last_change_ns=start_ns;
    double last_r=-1,last_g=-1,last_b=-1;

    uint8_t prev_fc[3] = {
        attr_ptrs[0][0],
        static_cast<uint8_t>(triple ? attr_ptrs[1][0] : 0),
        static_cast<uint8_t>(triple ? attr_ptrs[2][0] : 0)
    };

    // FPS EMA
    uint64_t last_frame_ns = start_ns;
    double fps_ema = 0.0;
    const double alpha = 0.2;

    while(g_run){
        // Adaptive wait: spin ≤2ms, then 1ms nap, repeat until counter changes
        uint16_t s0 = sum_fc();
        for(;;){
            bool changed = false;
            uint64_t spin_start = now_ns();
            while(((now_ns()-spin_start) < 2'000'000ULL)) { // 2 ms
                if(sum_fc()!=s0){ changed=true; break; }
                short_relax();
            }
            if(changed) break;
            nanosnooze(); // gentle 1ms sleep to keep CPU tiny
            if(sum_fc()!=s0) break;
            if(!g_run) break;
        }
        if(!g_run) break;

        uint8_t curr_fc[3] = {
            attr_ptrs[0][0],
            static_cast<uint8_t>(triple ? attr_ptrs[1][0] : 0),
            static_cast<uint8_t>(triple ? attr_ptrs[2][0] : 0)
        };
        int buf = choose_active_idx(prev_fc, curr_fc);
        prev_fc[0]=curr_fc[0]; prev_fc[1]=curr_fc[1]; prev_fc[2]=curr_fc[2];

        const volatile uint8_t* pix = base + fb_off(large,(uint8_t)buf) + header_len;

        const int xs=kStep, ys=kStep;
        uint64_t rs=0,gs=0,bs=0,n=0;
        uint32_t hsh=2166136261u;

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
        } else {
            for(unsigned y=0;y<height;y+=ys){
                const volatile uint8_t* row=pix + (size_t)y*line;
                for(unsigned x=0;x<width;x+=xs){
                    const volatile uint8_t* p=row + (size_t)x*2;
                    uint8_t r,g,b; load_rgb565(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        }

        unsigned cx=width/2, cy=height/2;
        uint8_t Rc=0,Gc=0,Bc=0;
        if(fmt==RGB24){
            const volatile uint8_t* p=pix + (size_t)cy*line + (size_t)cx*3; Rc=p[0]; Gc=p[1]; Bc=p[2];
        } else if(fmt==RGBA32){
            const volatile uint8_t* p=pix + (size_t)cy*line + (size_t)cx*4; Rc=p[0]; Gc=p[1]; Bc=p[2];
        } else {
            const volatile uint8_t* p=pix + (size_t)cy*line + (size_t)cx*2; uint8_t r,g,b; load_rgb565(p,r,g,b); Rc=r;Gc=g;Bc=b;
        }

        uint64_t t_ns=now_ns();
        double r_avg = n? (double)rs/n : 0.0;
        double g_avg = n? (double)gs/n : 0.0;
        double b_avg = n? (double)bs/n : 0.0;

        static double last_r=-1,last_g=-1,last_b=-1;
        if(first){
            last_hash=hsh; last_change_ns=t_ns; last_r=r_avg; last_g=g_avg; last_b=b_avg; first=false;
        } else if(hsh!=last_hash){
            double dr=fabs(r_avg-last_r), dg=fabs(g_avg-last_g), db=fabs(b_avg-last_b);
            if((dr+dg+db) >= kTolerance){ last_change_ns=t_ns; last_hash=hsh; last_r=r_avg; last_g=g_avg; last_b=b_avg; }
            else { last_hash=hsh; }
        }

        // instantaneous + smoothed FPS
        double frame_dt = (t_ns - last_frame_ns) / 1e9;
        last_frame_ns = t_ns;
        double fps_inst = frame_dt > 0 ? 1.0 / frame_dt : 0.0;
        fps_ema = (fps_ema==0.0) ? fps_inst : (0.2*fps_inst + 0.8*fps_ema);

        double unchanged_s=(t_ns-last_change_ns)/1e9;
        double elapsed_s  =(t_ns-start_ns)/1e9;
        char tbuf[16]; fmt_hms(elapsed_s,tbuf,sizeof(tbuf));
        unsigned R=(unsigned)r_avg, G=(unsigned)g_avg, B=(unsigned)b_avg;

        printf("time=%s  unchanged=%.3f  fps=%.1f  sig=0x%08X  avg_rgb=#%02X%02X%02X  center_rgb=#%02X%02X%02X\n",
               tbuf, unchanged_s, fps_ema, hsh, R,G,B, Rc,Gc,Bc);
        fflush(stdout);
    }

    munmap((void*)base,MAP_LEN); close(fd); return 0;
}
