// mister_peeper.cpp
//
// Minimal per-frame sampler for MiSTer scaler output.
// - Reads the scaler header from 0x20000000 (ASCAL) via /dev/mem
// - Tracks an "unchanged" timer by comparing a stable grid of sampled pixels
// - Reports the dominant color over a sparse grid (fast 5-6-5 histogram)
// - Auto-detects 16-bit RGB565 ordering (RGB/BGR, LE/BE) and re-detects on core/geometry change
// - Low CPU usage via gentle polling
//
// Console output (stdout, one line per frame):
//   time=HH:MM:SS  unchanged=secs  dom_rgb=#RRGGBB (Name)
//
// Build (example):
//   g++ -O3 -march=native -fno-exceptions -fno-rtti -Wall -Wextra -o mister_peeper mister_peeper.cpp
//
// Run:
//   sudo ./mister_peeper
//
// Notes:
// - 24/32-bit paths assume RGB byte order as exposed by the scaler.
// - 16-bit paths auto-detect the correct loader once per geometry change.
// - One-line info messages (chosen 565 variant, scaler changes) are printed to stderr.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <climits>
#include <algorithm> // for std::max
#include <vector>

// ---------- Tunables ----------
static constexpr int    kStep            = 2;                 // sampling grid step (pixels)
static constexpr size_t FB_BASE_ADDRESS  = 0x20000000u;        // MiSTer scaler base (ASCAL)
static constexpr size_t MAP_LEN          = 2048u * 1024u * 12u;// ~24 MiB mapping window
static constexpr int    kPollMs          = 10;                 // ~100 Hz idle polling

enum ScalerPixelFormat : uint8_t { RGB16 = 0, RGB24 = 1, RGBA32 = 2 };

// ---------- Scaler header ----------
#pragma pack(push,1)
struct FbHeader {
    uint8_t  ty;                // 0x01
    uint8_t  pixel_fmt;         // 0,1,2
    uint16_t header_len_be;
    uint16_t attributes_be;     // bit4 triple; bits7..5 frame counter
    uint16_t width_be;
    uint16_t height_be;
    uint16_t line_be;           // stride in bytes
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

// ---------- Runtime ----------
static volatile bool g_run=true;
static void on_sig(int){ g_run=false; }

static inline uint64_t now_ns(){
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
    return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec;
}
static inline void fmt_hms(double s,char* out,size_t n){
    int S=(int)s; int H=S/3600; int M=(S%3600)/60; S%=60;
    snprintf(out,n,"%02d:%02d:%02d",H,M,S);
}

// ---------- Nearest color name (tiny palette for readability) ----------
struct NamedColor { const char* name; uint8_t r,g,b; };
static constexpr NamedColor kPalette[] = {
    {"Black",0,0,0},{"White",255,255,255},{"Red",255,0,0},{"Lime",0,255,0},
    {"Blue",0,0,255},{"Yellow",255,255,0},{"Cyan",0,255,255},{"Magenta",255,0,255},
    {"Silver",192,192,192},{"Gray",128,128,128},{"Maroon",128,0,0},{"Olive",128,128,0},
    {"Green",0,128,0},{"Purple",128,0,128},{"Teal",0,128,128},{"Navy",0,0,128},
    {"Orange",255,165,0},{"Pink",255,192,203},{"Brown",165,42,42},{"Gold",255,215,0}
};
static inline const char* nearest_color_name(uint8_t r,uint8_t g,uint8_t b){
    int best_i = 0;
    int best_d = INT_MAX;
    for(size_t i=0;i<sizeof(kPalette)/sizeof(kPalette[0]);++i){
        int dr = (int)r - (int)kPalette[i].r;
        int dg = (int)g - (int)kPalette[i].g;
        int db = (int)b - (int)kPalette[i].b;
        int d = dr*dr + dg*dg + db*db;
        if(d < best_d){ best_d = d; best_i = (int)i; }
    }
    return kPalette[best_i].name;
}

// ---------- 16-bit pixel loaders ----------
static inline void load_rgb565_LE(const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){
    uint16_t v=(uint16_t)p[0]|((uint16_t)p[1]<<8);
    r=(uint8_t)(((v>>11)&0x1F)*255/31);
    g=(uint8_t)(((v>>5 )&0x3F)*255/63);
    b=(uint8_t)(( v      &0x1F)*255/31);
}
static inline void load_rgb565_BE(const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){
    uint16_t v=((uint16_t)p[0]<<8)|(uint16_t)p[1];
    r=(uint8_t)(((v>>11)&0x1F)*255/31);
    g=(uint8_t)(((v>>5 )&0x3F)*255/63);
    b=(uint8_t)(( v      &0x1F)*255/31);
}
static inline void load_bgr565_LE(const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){
    uint16_t v=(uint16_t)p[0]|((uint16_t)p[1]<<8);
    b=(uint8_t)(((v>>11)&0x1F)*255/31);
    g=(uint8_t)(((v>>5 )&0x3F)*255/63);
    r=(uint8_t)(( v      &0x1F)*255/31);
}
static inline void load_bgr565_BE(const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){
    uint16_t v=((uint16_t)p[0]<<8)|(uint16_t)p[1];
    b=(uint8_t)(((v>>11)&0x1F)*255/31);
    g=(uint8_t)(((v>>5 )&0x3F)*255/63);
    r=(uint8_t)(( v      &0x1F)*255/31);
}

using Rgb16Loader = void (*)(const volatile uint8_t*,uint8_t&,uint8_t&,uint8_t&);
using PixelLoader = void (*)(const volatile uint8_t*,uint8_t&,uint8_t&,uint8_t&);

static inline void load_rgb24(const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; }
static inline void load_rgba32(const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; }

static void build_sample_offsets(std::vector<size_t>& offs, uint16_t w, uint16_t h,
                                 uint16_t line, size_t bpp){
    offs.clear();
    for(unsigned y=0; y<h; y+=kStep){
        size_t row = (size_t)y * line;
        for(unsigned x=0; x<w; x+=kStep){
            offs.push_back(row + (size_t)x * bpp);
        }
    }
}
// ---------- simple 16-bit autodetect ----------
static Rgb16Loader choose_rgb16_loader(const volatile uint8_t* pix,
                                       uint16_t w, uint16_t h, uint16_t line){
    struct Cand { const char* name; Rgb16Loader fn; } C[4] = {
        {"RGB565-LE", load_rgb565_LE}, {"RGB565-BE", load_rgb565_BE},
        {"BGR565-LE", load_bgr565_LE}, {"BGR565-BE", load_bgr565_BE}
    };
    double best = -1.0; int bi = 0; int used = 0;
    for(int i=0;i<4;i++){
        double mean[3]={0,0,0}, m2[3]={0,0,0}; uint64_t n=0;
        for(unsigned y=0; y<h && n<2048; y+=32){
            const volatile uint8_t* row = pix + (size_t)y*line;
            for(unsigned x=0; x<w && n<2048; x+=32){
                const volatile uint8_t* p = row + (size_t)x*2;
                uint8_t r,g,b; C[i].fn(p,r,g,b);
                double vals[3]={ (double)r, (double)g, (double)b };
                ++n;
                for(int c=0;c<3;c++){ double d=vals[c]-mean[c]; mean[c]+=d/n; m2[c]+=d*(vals[c]-mean[c]); }
            }
        double var = (m2[0]+m2[1]+m2[2])/(double)(n>1? n-1:1);
        if(var>best){ best=var; bi=i; used=(int)n; }
        }
    }
    fprintf(stderr,"info=rgb16_loader variant=%s samples=%d\n", C[bi].name, used);
    return C[bi].fn;
}

// ---------- 5-6-5 histogram with epoch trick ----------
static uint32_t g_epoch = 1;
static uint32_t g_stamp[65536];
static uint16_t g_count[65536];

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
    uint16_t header_len=be16(h.header_len_be);
    uint16_t width     =be16(h.width_be);
    uint16_t height    =be16(h.height_be);
    uint16_t line      =be16(h.line_be);
    ScalerPixelFormat fmt=(ScalerPixelFormat)h.pixel_fmt;
    bool triple = triple_buffered(h.attributes_be);

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

    // Keep last-seen geometry to detect core/scaler changes
    uint16_t last_width  = width;
    uint16_t last_height = height;
    uint16_t last_line   = line;
    uint8_t  last_fmt    = (uint8_t)fmt;
    uint16_t last_header_len = header_len;

    // Sampling offsets and previous sample storage
    size_t bytes_per_pixel = (fmt==RGB16?2:(fmt==RGB24?3:4));
    std::vector<size_t> sample_offsets;
    build_sample_offsets(sample_offsets,last_width,last_height,last_line,bytes_per_pixel);
    std::vector<uint32_t> prev_samples(sample_offsets.size(),0);
    bool prev_valid=false;
    uint64_t start_ns=now_ns(), last_change_ns=start_ns;

    // Previous per-buffer counters
    uint8_t prev_fc[3] = {
        attr_ptrs[0][0],
        static_cast<uint8_t>(triple ? attr_ptrs[1][0] : 0),
        static_cast<uint8_t>(triple ? attr_ptrs[2][0] : 0)
    };

    // RGB16 loader (auto-detected on first applicable frame; re-detected on scaler change)
    Rgb16Loader rgb16_loader = nullptr;

    while(g_run){
        // Wait for next frame (low wake)
        uint16_t s0 = attr_ptrs[0][0] + (triple ? (uint16_t)(attr_ptrs[1][0]+attr_ptrs[2][0]) : 0);
        while(g_run){
            uint16_t s1 = attr_ptrs[0][0] + (triple ? (uint16_t)(attr_ptrs[1][0]+attr_ptrs[2][0]) : 0);
            if(s1!=s0) break;
            usleep((useconds_t)(kPollMs*1000));
        }
        if(!g_run) break;

        // Re-read header to detect core/scaler changes
        FbHeader hc{}; memcpy(&hc,(const void*)base,sizeof(FbHeader));
        uint16_t cur_header_len=be16(hc.header_len_be);
        uint16_t cur_width     =be16(hc.width_be);
        uint16_t cur_height    =be16(hc.height_be);
        uint16_t cur_line      =be16(hc.line_be);
        uint8_t  cur_fmt_u8    = hc.pixel_fmt;
        ScalerPixelFormat cur_fmt = (ScalerPixelFormat)cur_fmt_u8;
        bool     cur_triple    = triple_buffered(hc.attributes_be);

        // If geometry/format changed: update and reset 16-bit loader
        if(cur_width!=last_width || cur_height!=last_height ||
           cur_line!=last_line || cur_fmt_u8!=last_fmt ||
           cur_header_len!=last_header_len || cur_triple!=triple)
        {
            last_width = cur_width; last_height = cur_height; last_line = cur_line;
            last_fmt = cur_fmt_u8; last_header_len = cur_header_len; triple = cur_triple;
            fmt = cur_fmt;
            // Recompute triple buffer size flag
            large = triple && !hdr_ok(0x00200000u) && hdr_ok(0x00800000u);

            // Reset rgb16 autodetect
            rgb16_loader = nullptr;

            // Rebuild sample offsets and reset history
            bytes_per_pixel = (fmt==RGB16?2:(fmt==RGB24?3:4));
            build_sample_offsets(sample_offsets,last_width,last_height,last_line,bytes_per_pixel);
            prev_samples.assign(sample_offsets.size(),0);
            prev_valid=false;
            fprintf(stderr,"info=scaler_changed w=%u h=%u line=%u fmt=%u triple=%u\n",
                    (unsigned)cur_width,(unsigned)cur_height,(unsigned)cur_line,
                    (unsigned)cur_fmt_u8,(unsigned)cur_triple);
        }

        // Pick active buffer (the one whose frame counter advanced, modulo 256)
        uint8_t curr_fc[3] = {
            attr_ptrs[0][0],
            static_cast<uint8_t>(triple ? attr_ptrs[1][0] : 0),
            static_cast<uint8_t>(triple ? attr_ptrs[2][0] : 0)
        };
        int buf = 0;
        if(triple){
            uint8_t diff0 = static_cast<uint8_t>(curr_fc[0] - prev_fc[0]);
            uint8_t diff1 = static_cast<uint8_t>(curr_fc[1] - prev_fc[1]);
            uint8_t diff2 = static_cast<uint8_t>(curr_fc[2] - prev_fc[2]);
            buf = (diff1 > diff0) ? 1 : 0;
            uint8_t best = (buf == 1) ? diff1 : diff0;
            if(diff2 > best) buf = 2;
        }
        prev_fc[0]=curr_fc[0]; prev_fc[1]=curr_fc[1]; prev_fc[2]=curr_fc[2];

        // Address of active frame pixels
        const volatile uint8_t* pix = base + fb_off(large,(uint8_t)buf) + last_header_len;

        if(fmt==RGB16 && !rgb16_loader){
            rgb16_loader = choose_rgb16_loader(pix, last_width, last_height, last_line);
        }
        PixelLoader loader = (fmt==RGB24) ? load_rgb24 :
                             (fmt==RGBA32) ? load_rgba32 : rgb16_loader;

        uint32_t const epoch = ++g_epoch; // histogram epoch
        uint32_t mode_key = 0; uint16_t mode_count = 0;
        bool changed=false;

        for(size_t i=0;i<sample_offsets.size();++i){
            const volatile uint8_t* p = pix + sample_offsets[i];
            uint8_t r,g,b; loader(p,r,g,b);
            uint32_t key = ((uint32_t)(r>>3)<<11) | ((uint32_t)(g>>2)<<5) | (uint32_t)(b>>3);
            if(g_stamp[key]!=epoch){ g_stamp[key]=epoch; g_count[key]=1; }
            else { uint16_t c = ++g_count[key]; if(c>mode_count){ mode_count=c; mode_key=key; } }
            if(mode_count==0){ mode_count=1; mode_key=key; }

            uint32_t color=((uint32_t)r<<16)|((uint32_t)g<<8)|b;
            if(prev_valid && color!=prev_samples[i]) changed=true;
            prev_samples[i]=color;
        }

        uint64_t t_ns=now_ns();
        if(!prev_valid){ prev_valid=true; changed=true; }
        if(changed) last_change_ns=t_ns;

        // Expand 5-6-5 bin to 8-bit for output
        uint8_t Rd = (uint8_t)(((mode_key >> 11) & 0x1F) * 255 / 31);
        uint8_t Gd = (uint8_t)(((mode_key >> 5 ) & 0x3F) * 255 / 63);
        uint8_t Bd = (uint8_t)((  mode_key        & 0x1F) * 255 / 31);

        // Output
        double unchanged_s=(t_ns-last_change_ns)/1e9;
        double elapsed_s  =(t_ns-start_ns)/1e9;
        char tbuf[16]; fmt_hms(elapsed_s,tbuf,sizeof(tbuf));
        const char* dom_name = nearest_color_name(Rd,Gd,Bd);

        printf("time=%s  unchanged=%.3f  rgb=#%02X%02X%02X (%s)\n",
               tbuf, unchanged_s, Rd, Gd, Bd, dom_name);
        fflush(stdout);
    }

    munmap((void*)base,MAP_LEN); close(fd); return 0;
}
