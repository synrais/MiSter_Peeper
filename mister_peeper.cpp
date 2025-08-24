// mister_peeper.cpp
//
// Minimal per-frame sampler for MiSTer scaler output.
// - Reads the scaler header from 0x20000000 (ASCAL) via /dev/mem
// - Tracks an "unchanged" timer using a frame hash + histogram bin delta
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
#include <algorithm>

// ---------- Tunables ----------
static constexpr int    kStep            = 8;                 // sampling grid step (pixels)
static constexpr size_t FB_BASE_ADDRESS  = 0x20000000u;        // MiSTer scaler base (ASCAL)
static constexpr size_t MAP_LEN          = 2048u * 1024u * 12u;// ~24 MiB mapping window
static constexpr int    kPollMs          = 10;                 // ~100 Hz idle polling
static constexpr int    kChangedBinThreshold = 50;             // # of bins changed to trigger "changed"

enum ScalerPixelFormat : uint8_t { RGB16 = 0, RGB24 = 1, RGBA32 = 2 };

// ---------- Scaler header ----------
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

// ---------- Runtime ----------
static volatile bool g_run=true;
static void on_sig(int){ g_run=false; }

static inline uint64_t now_ns(){
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
    return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec;
}
static inline uint32_t hash_rgb(uint32_t h,uint8_t r,uint8_t g,uint8_t b){
    h^=((uint32_t)r<<16)^((uint32_t)g<<8)^(uint32_t)b;
    h^=h<<13; h^=h>>17; h^=h<<5; return h;
}
static inline void fmt_hms(double s,char* out,size_t n){
    int S=(int)s; int H=S/3600; int M=(S%3600)/60; S%=60;
    snprintf(out,n,"%02d:%02d:%02d",H,M,S);
}

// ---------- Color names ----------
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
static inline void load_rgb565_LE(const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ uint16_t v=(uint16_t)p[0]|((uint16_t)p[1]<<8); r=(uint8_t)(((v>>11)&0x1F)*255/31); g=(uint8_t)(((v>>5 )&0x3F)*255/63); b=(uint8_t)(( v      &0x1F)*255/31); }
static inline void load_rgb565_BE(const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ uint16_t v=((uint16_t)p[0]<<8)|(uint16_t)p[1]; r=(uint8_t)(((v>>11)&0x1F)*255/31); g=(uint8_t)(((v>>5 )&0x3F)*255/63); b=(uint8_t)(( v      &0x1F)*255/31); }
static inline void load_bgr565_LE(const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ uint16_t v=(uint16_t)p[0]|((uint16_t)p[1]<<8); b=(uint8_t)(((v>>11)&0x1F)*255/31); g=(uint8_t)(((v>>5 )&0x3F)*255/63); r=(uint8_t)(( v      &0x1F)*255/31); }
static inline void load_bgr565_BE(const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ uint16_t v=((uint16_t)p[0]<<8)|(uint16_t)p[1]; b=(uint8_t)(((v>>11)&0x1F)*255/31); g=(uint8_t)(((v>>5 )&0x3F)*255/63); r=(uint8_t)(( v      &0x1F)*255/31); }
using Rgb16Loader = void (*)(const volatile uint8_t*,uint8_t&,uint8_t&,uint8_t&);

// ---------- robust 16-bit autodetect (kept exactly as original) ----------
struct Stats { double mean[3]; double m2[3]; uint8_t minv[3]; uint8_t maxv[3]; uint64_t n; };
static inline void stats_init(Stats&s){ s.mean[0]=s.mean[1]=s.mean[2]=0.0; s.m2[0]=s.m2[1]=s.m2[2]=0.0; s.minv[0]=s.minv[1]=s.minv[2]=255; s.maxv[0]=s.maxv[1]=s.maxv[2]=0; s.n=0; }
static inline void stats_push(Stats&s,uint8_t r,uint8_t g,uint8_t b){ uint8_t v[3]={r,g,b}; s.n++; for(int i=0;i<3;i++){ double x=v[i]; double d=x - s.mean[i]; s.mean[i]+=d/(double)s.n; s.m2[i]+=d*(x-s.mean[i]); if(v[i]<s.minv[i]) s.minv[i]=v[i]; if(v[i]>s.maxv[i]) s.maxv[i]=v[i]; } }
static inline double stats_var(const Stats&s,int i){ return (s.n>1) ? (s.m2[i]/(double)(s.n-1)) : 0.0; }
static Rgb16Loader choose_rgb16_loader(const volatile uint8_t* pix,uint16_t w,uint16_t h,uint16_t line){
    struct Cand { const char* name; Rgb16Loader fn; } C[4] = { {"RGB565-LE", load_rgb565_LE}, {"RGB565-BE", load_rgb565_BE}, {"BGR565-LE", load_bgr565_LE}, {"BGR565-BE", load_bgr565_BE} };
    const int off_table[][2] = { {0,0},{8,8},{4,12},{12,4},{2,2},{6,10},{10,6} };
    Stats S[4]; for(int i=0;i<4;i++) stats_init(S[i]); int samples=0;
    for(auto& off:off_table){ int ox=off[0],oy=off[1]; for(unsigned y=oy;y<h;y+=32){ const volatile uint8_t* row=pix+(size_t)y*line; for(unsigned x=ox;x<w;x+=32){ const volatile uint8_t* p=row+(size_t)x*2; uint8_t r,g,b; for(int i=0;i<4;i++){ C[i].fn(p,r,g,b); stats_push(S[i],r,g,b);} if(++samples>1200) break;} if(samples>1200) break;} }
    auto score=[&](const Stats& st){ double vr=stats_var(st,0),vg=stats_var(st,1),vb=stats_var(st,2); double c=(vr-vg)*(vr-vg)+(vg-vb)*(vg-vb)+(vb-vr)*(vb-vr); double spanR=st.maxv[0]-st.minv[0], spanG=st.maxv[1]-st.minv[1], spanB=st.maxv[2]-st.minv[2]; double flat=(spanR<2)+(spanG<2)+(spanB<2); return c-1e6*flat; };
    double best=-1e300; int bi=0; for(int i=0;i<4;i++){ double sc=score(S[i]); if(sc>best){best=sc;bi=i;}} fprintf(stderr,"info=rgb16_loader variant=%s samples=%llu\n",C[bi].name,(unsigned long long)S[bi].n); return C[bi].fn;
}

// ---------- 5-6-5 histogram ----------
static uint32_t g_epoch = 1;
static uint32_t g_stamp[65536];
static uint16_t g_count[65536];

int main(){
    signal(SIGINT,on_sig); signal(SIGTERM,on_sig);

    int fd=open("/dev/mem",O_RDONLY|O_SYNC);
    if(fd<0){ perror("open(/dev/mem)"); return 1; }
    void* map=mmap(nullptr,MAP_LEN,PROT_READ,MAP_SHARED,fd,FB_BASE_ADDRESS);
    if(map==MAP_FAILED){ perror("mmap"); close(fd); return 1; }
    volatile uint8_t* base=(volatile uint8_t*)map;

    FbHeader h{}; memcpy(&h,(const void*)base,sizeof(FbHeader));
    if(h.ty!=0x01){ fprintf(stderr,"error=header_not_found ty=%u\n",(unsigned)h.ty); munmap((void*)base,MAP_LEN); close(fd); return 3; }
    uint16_t header_len=be16(h.header_len_be), width=be16(h.width_be), height=be16(h.height_be), line=be16(h.line_be);
    ScalerPixelFormat fmt=(ScalerPixelFormat)h.pixel_fmt; bool triple=triple_buffered(h.attributes_be);

    auto hdr_ok=[&](size_t off){ if(off+sizeof(FbHeader)>MAP_LEN) return false; FbHeader t{}; memcpy(&t,(const void*)(base+off),sizeof(FbHeader)); return t.ty==0x01 && t.pixel_fmt<=2; };
    bool large=triple&&!hdr_ok(0x00200000u)&&hdr_ok(0x00800000u);

    volatile const uint8_t* attr_ptrs[3]={ base+5, base+fb_off(large,1)+5, base+fb_off(large,2)+5 };

    uint16_t last_width=width,last_height=height,last_line=line; uint8_t last_fmt=(uint8_t)fmt; uint16_t last_header_len=header_len;
    auto load_rgb24=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };
    auto load_rgba32=[](const volatile uint8_t*p,uint8_t&r,uint8_t&g,uint8_t&b){ r=p[0]; g=p[1]; b=p[2]; };

    uint32_t last_hash=0; bool first=true; uint64_t start_ns=now_ns(), last_change_ns=start_ns; uint32_t prev_epoch=0;
    uint8_t prev_fc[3]={ attr_ptrs[0][0], static_cast<uint8_t>(triple?attr_ptrs[1][0]:0), static_cast<uint8_t>(triple?attr_ptrs[2][0]:0) };
    Rgb16Loader rgb16_loader=nullptr;

    while(g_run){
        // wait frame
        uint16_t s0=attr_ptrs[0][0]+(triple?(uint16_t)(attr_ptrs[1][0]+attr_ptrs[2][0]):0);
        while(g_run){ uint16_t s1=attr_ptrs[0][0]+(triple?(uint16_t)(attr_ptrs[1][0]+attr_ptrs[2][0]):0); if(s1!=s0) break; usleep((useconds_t)(kPollMs*1000)); }
        if(!g_run) break;

        // detect format/geometry changes
        FbHeader hc{}; memcpy(&hc,(const void*)base,sizeof(FbHeader));
        uint16_t cur_header_len=be16(hc.header_len_be), cur_width=be16(hc.width_be), cur_height=be16(hc.height_be), cur_line=be16(hc.line_be);
        uint8_t cur_fmt_u8=hc.pixel_fmt; ScalerPixelFormat cur_fmt=(ScalerPixelFormat)cur_fmt_u8; bool cur_triple=triple_buffered(hc.attributes_be);
        if(cur_width!=last_width||cur_height!=last_height||cur_line!=last_line||cur_fmt_u8!=last_fmt||cur_header_len!=last_header_len||cur_triple!=triple){
            last_width=cur_width; last_height=cur_height; last_line=cur_line; last_fmt=cur_fmt_u8; last_header_len=cur_header_len; triple=cur_triple; large=triple&&!hdr_ok(0x00200000u)&&hdr_ok(0x00800000u);
            rgb16_loader=nullptr; last_hash=0; first=true;
            fprintf(stderr,"info=scaler_changed w=%u h=%u line=%u fmt=%u triple=%u\n",(unsigned)cur_width,(unsigned)cur_height,(unsigned)cur_line,(unsigned)cur_fmt_u8,(unsigned)cur_triple);
        }

        // pick buffer
        uint8_t curr_fc[3]={ attr_ptrs[0][0], static_cast<uint8_t>(triple?attr_ptrs[1][0]:0), static_cast<uint8_t>(triple?attr_ptrs[2][0]:0) };
        int idx=0; if(triple){ for(int i=0;i<3;i++) if(curr_fc[i]!=prev_fc[i]){ idx=i; break; } if(curr_fc[1]>=curr_fc[idx]) idx=1; if(curr_fc[2]>=curr_fc[idx]) idx=2; }
        int buf=idx; prev_fc[0]=curr_fc[0]; prev_fc[1]=curr_fc[1]; prev_fc[2]=curr_fc[2];

        const volatile uint8_t* pix=base+fb_off(large,(uint8_t)buf)+last_header_len;
        const int xs=kStep,ys=kStep; uint32_t hsh=2166136261u;

        uint32_t epoch=++g_epoch; uint32_t mode_key=0; uint16_t mode_count=0;

        if(fmt==RGB24){ for(unsigned y=0;y<last_height;y+=ys){ const volatile uint8_t* row=pix+(size_t)y*last_line; for(unsigned x=0;x<last_width;x+=xs){ const volatile uint8_t* p=row+(size_t)x*3; uint8_t r=p[0],g=p[1],b=p[2]; hsh=hash_rgb(hsh,r,g,b); uint32_t key=((uint32_t)(r>>3)<<11)|((uint32_t)(g>>2)<<5)|(uint32_t)(b>>3); if(g_stamp[key]!=epoch){ g_stamp[key]=epoch; g_count[key]=1;} else { uint16_t c=++g_count[key]; if(c>mode_count){mode_count=c;mode_key=key;}} if(mode_count==0){mode_count=1;mode_key=key;}}}}
        else if(fmt==RGBA32){ for(unsigned y=0;y<last_height;y+=ys){ const volatile uint8_t* row=pix+(size_t)y*last_line; for(unsigned x=0;x<last_width;x+=xs){ const volatile uint8_t* p=row+(size_t)x*4; uint8_t r=p[0],g=p[1],b=p[2]; hsh=hash_rgb(hsh,r,g,b); uint32_t key=((uint32_t)(r>>3)<<11)|((uint32_t)(g>>2)<<5)|(uint32_t)(b>>3); if(g_stamp[key]!=epoch){ g_stamp[key]=epoch; g_count[key]=1;} else { uint16_t c=++g_count[key]; if(c>mode_count){mode_count=c;mode_key=key;}} if(mode_count==0){mode_count=1;mode_key=key;}}}}
        else { if(!rgb16_loader) rgb16_loader=choose_rgb16_loader(pix,last_width,last_height,last_line); for(unsigned y=0;y<last_height;y+=ys){ const volatile uint8_t* row=pix+(size_t)y*last_line; for(unsigned x=0;x<last_width;x+=xs){ const volatile uint8_t* p=row+(size_t)x*2; uint8_t r,g,b; rgb16_loader(p,r,g,b); hsh=hash_rgb(hsh,r,g,b); uint32_t key=((uint32_t)(r>>3)<<11)|((uint32_t)(g>>2)<<5)|(uint32_t)(b>>3); if(g_stamp[key]!=epoch){ g_stamp[key]=epoch; g_count[key]=1;} else { uint16_t c=++g_count[key]; if(c>mode_count){mode_count=c;mode_key=key;}} if(mode_count==0){mode_count=1;mode_key=key;}}}}

        // Count how many bins changed compared to last frame
        int changed_bins=0;
        if(prev_epoch){ for(int i=0;i<65536;i++){ if(g_stamp[i]==epoch && g_stamp[i]!=prev_epoch) changed_bins++; } }
        prev_epoch=epoch;

        // Change detection: hash OR bin delta
        uint64_t t_ns=now_ns();
        bool frame_changed=(hsh!=last_hash)||(changed_bins>=kChangedBinThreshold);
        if(first){ last_hash=hsh; last_change_ns=t_ns; first=false; }
        else if(frame_changed){ last_change_ns=t_ns; last_hash=hsh; }

        uint8_t Rd=(uint8_t)(((mode_key>>11)&0x1F)*255/31);
        uint8_t Gd=(uint8_t)(((mode_key>>5 )&0x3F)*255/63);
        uint8_t Bd=(uint8_t)(( mode_key     &0x1F)*255/31);

        double unchanged_s=(t_ns-last_change_ns)/1e9; double elapsed_s=(t_ns-start_ns)/1e9; char tbuf[16]; fmt_hms(elapsed_s,tbuf,sizeof(tbuf));
        const char* dom_name=nearest_color_name(Rd,Gd,Bd);

        printf("time=%s  unchanged=%.3f  rgb=#%02X%02X%02X (%s)\n",tbuf,unchanged_s,Rd,Gd,Bd,dom_name);
        fflush(stdout);
    }

    munmap((void*)base,MAP_LEN); close(fd); return 0;
}
