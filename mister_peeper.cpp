// mister_peeper.cpp — minimal, fast screen checker for MiSTer scaler
// Output per line: time=HH:MM:SS  unchanged=secs  avg_rgb=#RRGGBB  center_rgb=#RRGGBB
// Only option: --sleep-us N  (default 5000 µs)

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

static constexpr int    kStep      = 16;     // sampling grid (fixed)
static constexpr double kTolerance = 3.0;    // L1 on avg RGB to count as "changed"
static       int        gSleepUs   = 5000;   // overridable via --sleep-us

// MiSTer scaler mapping
static constexpr size_t FB_BASE_ADDRESS = 0x20000000u;
static constexpr size_t MAP_LEN         = 2048u * 1024u * 12u; // ~24 MiB

enum ScalerPixelFormat : uint8_t { RGB16 = 0, RGB24 = 1, RGBA32 = 2 };

#pragma pack(push,1)
struct FbHeader {
    uint8_t  ty;                // 0x01
    uint8_t  pixel_fmt;         // 0,1,2
    uint16_t header_len_be;
    uint16_t attributes_be;     // bit4 triple-buffered; bits7..5 = frame counter
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

static inline uint64_t now_ns(){ struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts); return (uint64_t)ts.tv_sec*1000000000ull+ts.tv_nsec; }
static inline uint32_t hash_rgb(uint32_t h,uint8_t r,uint8_t g,uint8_t b){ h^=((uint32_t)r<<16)^((uint32_t)g<<8)^(uint32_t)b; h^=h<<13; h^=h>>17; h^=h<<5; return h; }
static inline void fmt_hms(double s,char* out,size_t n){ int S=(int)s; int H=S/3600; int M=(S%3600)/60; S%=60; snprintf(out,n,"%02d:%02d:%02d",H,M,S); }

int main(int argc,char**argv){
    // only --sleep-us
    for(int i=1;i<argc;i++){
        if(!strcmp(argv[i],"--sleep-us") && i+1<argc){ gSleepUs=atoi(argv[++i]); if(gSleepUs<0) gSleepUs=0; }
        else { fprintf(stderr,"Unknown option: %s (only --sleep-us allowed)\n", argv[i]); return 2; }
    }

    signal(SIGINT,on_sig); signal(SIGTERM,on_sig);

    // map scaler
    int fd=open("/dev/mem",O_RDONLY|O_SYNC);
    if(fd<0){ perror("open(/dev/mem)"); return 1; }
    void* map=mmap(nullptr,MAP_LEN,PROT_READ,MAP_SHARED,fd,FB_BASE_ADDRESS);
    if(map==MAP_FAILED){ perror("mmap"); close(fd); return 1; }
    volatile uint8_t* base=(volatile uint8_t*)map;

    // read header
    FbHeader h{}; memcpy(&h,(const void*)base,sizeof(FbHeader));
    if(h.ty!=0x01){ fprintf(stderr,"error=header_not_found ty=%u\n",(unsigned)h.ty); munmap((void*)base,MAP_LEN); close(fd); return 3; }

    const uint16_t header_len=be16(h.header_len_be);
    const uint16_t width     =be16(h.width_be);
    const uint16_t height    =be16(h.height_be);
    const uint16_t line      =be16(h.line_be);
    const ScalerPixelFormat fmt=(ScalerPixelFormat)h.pixel_fmt;
    const bool triple = triple_buffered(h.attributes_be);

    // detect small vs large triple buffer by probing headers
    auto hdr_ok=[&](size_t off){ if(off+sizeof(FbHeader)>MAP_LEN) return false; FbHeader t{}; memcpy(&t,(const void*)(base+off),sizeof(FbHeader)); return t.ty==0x01 && t.pixel_fmt<=2; };
    bool large = triple && !hdr_ok(0x00200000u) && hdr_ok(0x00800000u);

    // pointers
    const volatile uint8_t* pix0 = base + header_len;
    volatile const uint8_t* fc_ptrs[3] = {
        base + 5,
        base + fb_off(large,1) + 5,
        base + fb_off(large,2) + 5
    };
    auto fc_sum=[&](){ uint8_t s=fc_ptrs[0][0]; if(triple){ s=(uint8_t)(s + fc_ptrs[1][0] + fc_ptrs[2][0]); } return s; };

    // change detection state
    uint32_t last_hash=0; bool first=true;
    uint64_t start_ns=now_ns(), last_change_ns=start_ns;
    double last_r=-1,last_g=-1,last_b=-1;

    // pixel loaders
    auto load_rgb24=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgba32=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgb565=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ uint16_t v=(uint16_t)p[0]|((uint16_t)p[1]<<8); r=((v>>11)&0x1F)*255/31; g=((v>>5)&0x3F)*255/63; b=(v&0x1F)*255/31; };

    while(g_run){
        // light frame sync (keeps CPU/bus low but responsive)
        uint8_t f0=fc_sum();
        for(int i=0;i<2 && g_run;i++){
            if(fc_sum()!=f0) break;
            if(gSleepUs>0) usleep(gSleepUs);
        }
        if(!g_run) break;

        // sample sparse grid
        const int xs=kStep, ys=kStep;
        uint64_t rs=0,gs=0,bs=0,n=0;
        uint32_t hsh=2166136261u;

        if(fmt==RGB24){
            for(unsigned y=0;y<height;y+=ys){
                const volatile uint8_t* row=pix0 + (size_t)y*line;
                for(unsigned x=0;x<width;x+=xs){
                    const volatile uint8_t* p=row + (size_t)x*3;
                    uint8_t r,g,b; load_rgb24(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        } else if(fmt==RGBA32){
            for(unsigned y=0;y<height;y+=ys){
                const volatile uint8_t* row=pix0 + (size_t)y*line;
                for(unsigned x=0;x<width;x+=xs){
                    const volatile uint8_t* p=row + (size_t)x*4;
                    uint8_t r,g,b; load_rgba32(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        } else { // RGB16
            for(unsigned y=0;y<height;y+=ys){
                const volatile uint8_t* row=pix0 + (size_t)y*line;
                for(unsigned x=0;x<width;x+=xs){
                    const volatile uint8_t* p=row + (size_t)x*2;
                    uint8_t r,g,b; load_rgb565(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        }

        // averages
        double r_avg = n ? (double)rs/n : 0.0;
        double g_avg = n ? (double)gs/n : 0.0;
        double b_avg = n ? (double)bs/n : 0.0;
        unsigned R=(unsigned)r_avg, G=(unsigned)g_avg, B=(unsigned)b_avg;

        // center pixel
        unsigned cx=width/2, cy=height/2;
        uint8_t Rc=0,Gc=0,Bc=0;
        if(fmt==RGB24){
            const volatile uint8_t* p=pix0 + (size_t)cy*line + (size_t)cx*3; Rc=p[0]; Gc=p[1]; Bc=p[2];
        } else if(fmt==RGBA32){
            const volatile uint8_t* p=pix0 + (size_t)cy*line + (size_t)cx*4; Rc=p[0]; Gc=p[1]; Bc=p[2];
        } else {
            const volatile uint8_t* p=pix0 + (size_t)cy*line + (size_t)cx*2; uint8_t r,g,b; load_rgb565(p,r,g,b); Rc=r; Gc=g; Bc=b;
        }

        // change detection (hash + tolerance on avg)
        uint64_t t_ns=now_ns();
        if(first){
            last_hash=hsh; last_change_ns=t_ns; last_r=r_avg; last_g=g_avg; last_b=b_avg; first=false;
        } else if(hsh!=last_hash){
            double dr=fabs(r_avg-last_r), dg=fabs(g_avg-last_g), db=fabs(b_avg-last_b);
            if((dr+dg+db) >= kTolerance){
                last_change_ns=t_ns; last_hash=hsh; last_r=r_avg; last_g=g_avg; last_b=b_avg;
            }
        }

        // print minimal line
        double unchanged_s=(t_ns-last_change_ns)/1e9;
        double elapsed_s  =(t_ns-now_ns()+ (double)(t_ns - t_ns))/1e9; // avoid extra syscalls? (we'll compute fresh)
        elapsed_s = (now_ns()- (t_ns - (now_ns()-t_ns)))/1e9; // but that's silly; just compute from start
        elapsed_s = (now_ns() - start_ns)/1e9;
        char tbuf[16]; fmt_hms(elapsed_s,tbuf,sizeof(tbuf));
        printf("time=%s  unchanged=%.3f  avg_rgb=#%02X%02X%02X  center_rgb=#%02X%02X%02X\n",
               tbuf, unchanged_s, R,G,B, Rc,Gc,Bc);
        fflush(stdout);

        if(gSleepUs>0) usleep(gSleepUs);
    }

    munmap((void*)base,MAP_LEN);
    close(fd);
    return 0;
}
