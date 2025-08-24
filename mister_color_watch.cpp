// mister_color_watch.cpp
// Standalone color monitor for MiSTer scaler output
// Fixed defaults: step=16, tolerance=3.0
// Only option: --sleep-us N   (default 2500µs)

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

static constexpr int    kStepDefault      = 16;
static constexpr double kToleranceDefault = 3.0;
static       int        gSleepUs          = 2500;

// --- helpers ---
static volatile bool g_run = true;
static void sigint(int){ g_run = false; }

static inline uint64_t now_ns(){
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec;
}

static inline uint32_t hash_rgb(uint32_t h, uint8_t r, uint8_t g, uint8_t b){
    h ^= ((uint32_t)r << 16) ^ ((uint32_t)g << 8) ^ (uint32_t)b;
    h ^= h << 13; h ^= h >> 17; h ^= h << 5;
    return h;
}

// convert seconds to HH:MM:SS string
static void fmt_hms(double sec, char* out, size_t out_sz){
    int s = (int)sec;
    int h = s / 3600;
    int m = (s % 3600) / 60;
    s = s % 60;
    snprintf(out, out_sz, "%02d:%02d:%02d", h, m, s);
}

// --- MiSTer scaler header ---
static constexpr size_t FB_BASE_ADDRESS = 0x20000000u;
static constexpr size_t MAP_LEN         = 2048u * 1024u * 12u;

enum ScalerPixelFormat : uint8_t { RGB16 = 0, RGB24 = 1, RGBA32 = 2, INVALID=0xFF };

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

static inline uint16_t be16(uint16_t x){ return (uint16_t)((x>>8) | (x<<8)); }
static inline bool triple_buffered(uint16_t attr_be){ return (be16(attr_be) & (1u<<4)) != 0; }
static inline size_t fb_offset_for_index(bool triple_large, uint8_t index){
    if(index==0) return 0;
    if(triple_large) return (index==1) ? 0x00800000u : 0x01000000u;
    return (index==1) ? 0x00200000u : 0x00400000u;
}

int main(int argc, char** argv){
    // parse only --sleep-us
    for(int i=1;i<argc;i++){
        if(!strcmp(argv[i],"--sleep-us") && i+1<argc){
            gSleepUs = atoi(argv[++i]);
            if(gSleepUs < 0) gSleepUs = 0;
        } else {
            fprintf(stderr,"Unknown option: %s (only --sleep-us allowed)\n", argv[i]);
            return 2;
        }
    }

    signal(SIGINT, sigint); signal(SIGTERM, sigint);
    const int step = kStepDefault;
    const double tol = kToleranceDefault;

    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if(fd<0){ perror("open(/dev/mem)"); return 1; }
    void* map = mmap(nullptr, MAP_LEN, PROT_READ, MAP_SHARED, fd, FB_BASE_ADDRESS);
    if(map==MAP_FAILED){ perror("mmap"); close(fd); return 1; }
    auto* base = (volatile uint8_t*)map;

    FbHeader h{};
    memcpy((void*)&h, (const void*)base, sizeof(FbHeader));
    if(h.ty != 0x01){
        fprintf(stderr,"error=header_not_found ty=%u\n",(unsigned)h.ty);
        return 3;
    }

    const uint16_t header_len = be16(h.header_len_be);
    const uint16_t width  = be16(h.width_be);
    const uint16_t height = be16(h.height_be);
    const uint16_t line   = be16(h.line_be);
    const ScalerPixelFormat fmt = (ScalerPixelFormat)h.pixel_fmt;

    // pixel loaders
    auto load_rgb24 = [](const volatile uint8_t* p, uint8_t& r, uint8_t& g, uint8_t& b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgba32= [](const volatile uint8_t* p, uint8_t& r, uint8_t& g, uint8_t& b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgb565= [](const volatile uint8_t* p, uint8_t& r, uint8_t& g, uint8_t& b){
        uint16_t v = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
        r = (uint8_t)(((v >> 11) & 0x1F) * 255 / 31);
        g = (uint8_t)(((v >> 5)  & 0x3F) * 255 / 63);
        b = (uint8_t)(( v        & 0x1F) * 255 / 31);
    };

    fprintf(stdout,"info=detected fmt=%s size=%ux%u step=%d sleep_us=%d tol=%.1f\n",
        (fmt==RGB24?"RGB24":fmt==RGBA32?"RGBA32":fmt==RGB16?"RGB16":"INVALID"),
        (unsigned)width,(unsigned)height, step, gSleepUs, tol);
    fflush(stdout);

    // change detection state
    uint32_t last_hash = 0;
    bool first = true;
    uint64_t last_change_ns = now_ns();
    double last_r=-1,last_g=-1,last_b=-1;
    uint64_t start_ns = last_change_ns;

    while(g_run){
        const volatile uint8_t* pix0 = base + header_len;
        const int xs = step, ys = step;
        uint64_t rs=0, gs=0, bs=0, n=0;
        uint32_t hsh = 2166136261u;

        if(fmt==RGB24){
            for(unsigned y=0; y<height; y+=ys){
                const volatile uint8_t* row = pix0 + (size_t)y * line;
                for(unsigned x=0; x<width; x+=xs){
                    const volatile uint8_t* p = row + (size_t)x * 3;
                    uint8_t r,g,b; load_rgb24(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        } else if(fmt==RGBA32){
            for(unsigned y=0; y<height; y+=ys){
                const volatile uint8_t* row = pix0 + (size_t)y * line;
                for(unsigned x=0; x<width; x+=xs){
                    const volatile uint8_t* p = row + (size_t)x * 4;
                    uint8_t r,g,b; load_rgba32(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        } else if(fmt==RGB16){
            for(unsigned y=0; y<height; y+=ys){
                const volatile uint8_t* row = pix0 + (size_t)y * line;
                for(unsigned x=0; x<width; x+=xs){
                    const volatile uint8_t* p = row + (size_t)x * 2;
                    uint8_t r,g,b; load_rgb565(p,r,g,b);
                    rs+=r; gs+=g; bs+=b; n++; hsh=hash_rgb(hsh,r,g,b);
                }
            }
        }

        double r_avg = n ? (double)rs/n : 0.0;
        double g_avg = n ? (double)gs/n : 0.0;
        double b_avg = n ? (double)bs/n : 0.0;
        unsigned R=(unsigned)r_avg, G=(unsigned)g_avg, B=(unsigned)b_avg;

        unsigned cx = width/2, cy = height/2;
        uint8_t Rc=0,Gc=0,Bc=0;
        if(fmt==RGB24){
            const volatile uint8_t* p = pix0 + (size_t)cy*line + (size_t)cx*3;
            Rc=p[0]; Gc=p[1]; Bc=p[2];
        } else if(fmt==RGBA32){
            const volatile uint8_t* p = pix0 + (size_t)cy*line + (size_t)cx*4;
            Rc=p[0]; Gc=p[1]; Bc=p[2];
        } else {
            const volatile uint8_t* p = pix0 + (size_t)cy*line + (size_t)cx*2;
            uint8_t r,g,b; load_rgb565(p,r,g,b);
            Rc=r; Gc=g; Bc=b;
        }

        uint64_t t_ns = now_ns();
        if(first){
            last_hash = hsh;
            last_change_ns = t_ns;
            last_r=r_avg; last_g=g_avg; last_b=b_avg;
            first=false;
        } else {
            if(hsh != last_hash){
                double dr=fabs(r_avg-last_r), dg=fabs(g_avg-last_g), db=fabs(b_avg-last_b);
                if((dr+dg+db) >= tol){
                    last_change_ns = t_ns;
                    last_hash = hsh;
                    last_r=r_avg; last_g=g_avg; last_b=b_avg;
                }
            }
        }

        double unchanged_s = (t_ns - last_change_ns) / 1e9;
        double elapsed_s   = (t_ns - start_ns) / 1e9;
        char tbuf[32]; fmt_hms(elapsed_s,tbuf,sizeof(tbuf));

        printf("time=%s  unchanged=%.3f  avg_rgb=#%02X%02X%02X  center_rgb=#%02X%02X%02X\n",
               tbuf, unchanged_s, R,G,B, Rc,Gc,Bc);
        fflush(stdout);

        if(gSleepUs>0) usleep(gSleepUs);
    }

    munmap((void*)map, MAP_LEN);
    close(fd);
    return 0;
}
