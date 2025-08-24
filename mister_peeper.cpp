// mister_peeper.cpp — minimal screen change monitor for MiSTer
// Checks ~100 times/sec, prints:
//   time=HH:MM:SS  unchanged=secs  avg_rgb=#RRGGBB  avg_color=Name

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

static volatile bool g_run=true;
static void on_sig(int){ g_run=false; }

static inline uint16_t be16(uint16_t x){ return (uint16_t)((x>>8)|(x<<8)); }
static inline bool triple_buffered(uint16_t attr_be){ return (be16(attr_be) & (1u<<4)) != 0; }
static inline size_t fb_off(bool large, uint8_t idx){
    if(idx==0) return 0;
    return large ? (idx==1 ? 0x00800000u : 0x01000000u)
                 : (idx==1 ? 0x00200000u : 0x00400000u);
}

static inline uint64_t now_ns(){ struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts); return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec; }
static inline void fmt_hms(double s,char* out,size_t n){ int S=(int)s; int H=S/3600; int M=(S%3600)/60; S%=60; snprintf(out,n,"%02d:%02d:%02d",H,M,S); }

// Tiny mixer for change detection
static inline uint32_t hash_rgb(uint32_t h,uint8_t r,uint8_t g,uint8_t b){
    h^=((uint32_t)r<<16)^((uint32_t)g<<8)^(uint32_t)b; h^=h<<13; h^=h>>17; h^=h<<5; return h;
}

// Map avg RGB to a basic color name (black/white/gray + 6 hues + orange/purple)
static inline const char* avg_color_name(unsigned R, unsigned G, unsigned B){
    unsigned V = (R>G? (R>B?R:B) : (G>B?G:B));
    unsigned m = (R<G? (R<B?R:B) : (G<B?G:B));
    unsigned C = V - m;

    // Grayscale buckets
    if (V < 20) return "Black";
    if (C < 15) { // low chroma => gray scale
        if (V > 235) return "White";
        if (V >= 170) return "Light Gray";
        if (V >= 80)  return "Gray";
        return "Dark Gray";
    }

    // Hue in degrees (0..360)
    double h;
    if (V == R)      h = 60.0 * fmod(((G - B) / (double)C), 6.0);
    else if (V == G) h = 60.0 * (((B - R) / (double)C) + 2.0);
    else             h = 60.0 * (((R - G) / (double)C) + 4.0);
    if (h < 0) h += 360.0;

    // Basic names
    if (h < 15 || h >= 345) return "Red";
    if (h < 45)             return "Orange";
    if (h < 75)             return "Yellow";
    if (h < 165)            return "Green";
    if (h < 195)            return "Cyan";
    if (h < 255)            return "Blue";
    if (h < 300)            return "Purple";
    return "Magenta";
}

int main(){
    signal(SIGINT,on_sig); signal(SIGTERM,on_sig);

    int fd=open("/dev/mem",O_RDONLY|O_SYNC);
    if(fd<0){ perror("open(/dev/mem)"); return 1; }
    void* map=mmap(nullptr,MAP_LEN,PROT_READ,MAP_SHARED,fd,FB_BASE_ADDRESS);
    if(map==MAP_FAILED){ perror("mmap"); close(fd); return 1; }
    volatile uint8_t* base=(volatile uint8_t*)map;

    // Read header
    FbHeader h{}; memcpy(&h,(const void*)base,sizeof(FbHeader));
    if(h.ty!=0x01){
        fprintf(stderr,"error=header_not_found\n");
        munmap((void*)base,MAP_LEN); close(fd); return 3;
    }
    const uint16_t header_len=be16(h.header_len_be);
    const uint16_t width     =be16(h.width_be);
    const uint16_t height    =be16(h.height_be);
    const uint16_t line      =be16(h.line_be);
    const ScalerPixelFormat fmt=(ScalerPixelFormat)h.pixel_fmt;
    const bool triple = triple_buffered(h.attributes_be);

    // Detect small vs large triple once
    auto hdr_ok=[&](size_t off){
        if(off+sizeof(FbHeader)>MAP_LEN) return false;
        FbHeader t{}; memcpy(&t,(const void*)(base+off),sizeof(FbHeader));
        return t.ty==0x01 && t.pixel_fmt<=2;
    };
    bool large = triple && !hdr_ok(0x00200000u) && hdr_ok(0x00800000u);

    auto pixel_base=[&](uint8_t idx)->volatile const uint8_t*{
        return base + fb_off(large,idx) + header_len;
    };

    // Pixel loaders
    auto load_rgb24=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgba32=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgb565=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){
        uint16_t v=(uint16_t)p[0]|((uint16_t)p[1]<<8);
        r=(uint8_t)(((v>>11)&0x1F)*255/31);
        g=(uint8_t)(((v>>5)&0x3F)*255/63);
        b=(uint8_t)(( v     &0x1F)*255/31);
    };

    // Change detection (signature of sampled pixels)
    uint32_t last_hash=0; bool first=true;
    uint64_t start_ns=now_ns(), last_change_ns=start_ns;

    while(g_run){
        // sample buffer 0 (header updates first, safe for our use)
        const volatile uint8_t* pix = pixel_base(0);

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
        } else { // RGB565
            for(unsigned y=0;y<height;y+=ys){
                const volatile uint8_t* row=pix + (size_t)y*line;
                for(unsigned x=0;x<width;x+=xs){
                    const volatile uint8_t* p=row + (size_t)x*2;
                    uint8_t r,g,b; load_rgb565(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        }

        // change = signature flip (works for vector/mostly-black content)
        uint64_t t_ns=now_ns();
        if(first){ last_hash=hsh; last_change_ns=t_ns; first=false; }
        else if(hsh!=last_hash){ last_hash=hsh; last_change_ns=t_ns; }

        // averages + label
        double r_avg = n? (double)rs/n : 0.0;
        double g_avg = n? (double)gs/n : 0.0;
        double b_avg = n? (double)bs/n : 0.0;
        unsigned R=(unsigned)r_avg, G=(unsigned)g_avg, B=(unsigned)b_avg;

        double unchanged_s=(t_ns-last_change_ns)/1e9;
        double elapsed_s  =(t_ns-start_ns)/1e9;
        char tbuf[16]; fmt_hms(elapsed_s,tbuf,sizeof(tbuf));

        const char* name = avg_color_name(R,G,B);

        printf("time=%s  unchanged=%.3f  avg_rgb=#%02X%02X%02X  avg_color=%s\n",
               tbuf, unchanged_s, R,G,B, name);
        fflush(stdout);

        usleep(10000); // ~100 checks/sec
    }

    munmap((void*)base,MAP_LEN); close(fd); return 0;
}
