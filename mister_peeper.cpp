// mister_peeper.cpp — minimal, fast screen checker for MiSTer scaler
// Fixed at 100 Hz (10ms sleep), no options.
// Output per line: time=HH:MM:SS  unchanged=secs  avg_rgb=#RRGGBB  color=Name

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

static constexpr int    kStep      = 16;     // sampling grid
static constexpr double kTolerance = 3.0;    // change tolerance
static constexpr int    kSleepUs   = 10000;  // fixed 100Hz (10 ms)

// MiSTer scaler mapping
static constexpr size_t FB_BASE_ADDRESS = 0x20000000u;
static constexpr size_t MAP_LEN         = 2048u * 1024u * 12u; // ~24 MiB

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

static volatile bool g_run=true;
static void on_sig(int){ g_run=false; }

static inline uint64_t now_ns(){
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
    return (uint64_t)ts.tv_sec*1000000000ull+ts.tv_nsec;
}
static inline uint32_t hash_rgb(uint32_t h,uint8_t r,uint8_t g,uint8_t b){
    h^=((uint32_t)r<<16)^((uint32_t)g<<8)^(uint32_t)b;
    h^=h<<13; h^=h>>17; h^=h<<5; return h;
}
static inline void fmt_hms(double s,char* out,size_t n){
    int S=(int)s; int H=S/3600; int M=(S%3600)/60; S%=60;
    snprintf(out,n,"%02d:%02d:%02d",H,M,S);
}

// crude english color name
static const char* color_name(unsigned R,unsigned G,unsigned B){
    if(R<30 && G<30 && B<30) return "black";
    if(R>220 && G>220 && B>220) return "white";
    if(R>200 && G<80 && B<80) return "red";
    if(G>200 && R<80 && B<80) return "green";
    if(B>200 && R<80 && G<80) return "blue";
    if(R>200 && G>200 && B<80) return "yellow";
    if(R>200 && B>200 && G<80) return "magenta";
    if(G>200 && B>200 && R<80) return "cyan";
    return "grayish";
}

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

    const volatile uint8_t* pix0 = base + header_len;

    uint32_t last_hash=0; bool first=true;
    uint64_t start_ns=now_ns(), last_change_ns=start_ns;
    double last_r=-1,last_g=-1,last_b=-1;

    auto load_rgb24=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgba32=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgb565=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){
        uint16_t v=(uint16_t)p[0]|((uint16_t)p[1]<<8);
        r=((v>>11)&0x1F)*255/31;
        g=((v>>5)&0x3F)*255/63;
        b=(v&0x1F)*255/31;
    };

    while(g_run){
        usleep(kSleepUs);

        uint64_t rs=0,gs=0,bs=0,n=0;
        uint32_t hsh=2166136261u;

        if(fmt==RGB24){
            for(unsigned y=0;y<height;y+=kStep){
                const volatile uint8_t* row=pix0 + (size_t)y*line;
                for(unsigned x=0;x<width;x+=kStep){
                    const volatile uint8_t* p=row + (size_t)x*3;
                    uint8_t r,g,b; load_rgb24(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        } else if(fmt==RGBA32){
            for(unsigned y=0;y<height;y+=kStep){
                const volatile uint8_t* row=pix0 + (size_t)y*line;
                for(unsigned x=0;x<width;x+=kStep){
                    const volatile uint8_t* p=row + (size_t)x*4;
                    uint8_t r,g,b; load_rgba32(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        } else {
            for(unsigned y=0;y<height;y+=kStep){
                const volatile uint8_t* row=pix0 + (size_t)y*line;
                for(unsigned x=0;x<width;x+=kStep){
                    const volatile uint8_t* p=row + (size_t)x*2;
                    uint8_t r,g,b; load_rgb565(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        }

        double r_avg = n ? (double)rs/n : 0.0;
        double g_avg = n ? (double)gs/n : 0.0;
        double b_avg = n ? (double)bs/n : 0.0;
        unsigned R=(unsigned)r_avg, G=(unsigned)g_avg, B=(unsigned)b_avg;

        uint64_t t_ns=now_ns();
        if(first){ last_hash=hsh; last_change_ns=t_ns; last_r=r_avg; last_g=g_avg; last_b=b_avg; first=false; }
        else if(hsh!=last_hash){
            double dr=fabs(r_avg-last_r), dg=fabs(g_avg-last_g), db=fabs(b_avg-last_b);
            if((dr+dg+db) >= kTolerance){
                last_change_ns=t_ns; last_hash=hsh; last_r=r_avg; last_g=g_avg; last_b=b_avg;
            }
        }

        double unchanged_s=(t_ns-last_change_ns)/1e9;
        double elapsed_s=(t_ns-start_ns)/1e9;
        char tbuf[16]; fmt_hms(elapsed_s,tbuf,sizeof(tbuf));

        printf("time=%s  unchanged=%.3f  avg_rgb=#%02X%02X%02X  color=%s\n",
               tbuf, unchanged_s, R,G,B, color_name(R,G,B));
        fflush(stdout);
    }

    munmap((void*)base,MAP_LEN);
    close(fd);
    return 0;
}
