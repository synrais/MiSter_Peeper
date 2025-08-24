// mister_color_watch.cpp
// Standalone color monitor for MiSTer scaler output.
// Auto-detects width/height/stride/pixel format from the ASCAL header,
// waits on the header's frame counter, then samples pixels and reports
// average color + time since last visual change.
//
// Build on MiSTer HPS (ARM):
//   g++ -O3 -pipe -s -std=gnu++17 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard \
//       mister_color_watch.cpp -o mister_color_watch
//
// Run (needs root for /dev/mem):
//   sudo ./mister_color_watch
//
// Flags (optional):
//   --step N       sample every Nth pixel (default 16; 1 = full scan)
//   --sleep-us N   microseconds between polls (default 2500; 0 = no sleep)

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

static volatile bool g_run = true;
static void sigint(int){ g_run = false; }

static inline uint64_t now_ns(){
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec;
}

// tiny mixer for change detection (not cryptographic)
static inline uint32_t hash_rgb(uint32_t h, uint8_t r, uint8_t g, uint8_t b){
    h ^= ((uint32_t)r << 16) ^ ((uint32_t)g << 8) ^ (uint32_t)b;
    h ^= h << 13; h ^= h >> 17; h ^= h << 5;
    return h;
}

// --- Constants from MiSTer framebuffer helpers (public crate) ---
// Base address and triple-buffer offsets. RGB formats enum matches MiSTer.
static constexpr size_t FB_BASE_ADDRESS = 0x20000000u;           // scaler base
static constexpr size_t MAP_LEN        = 2048u * 1024u * 3u * 4u; // safe window (~24MB)
enum ScalerPixelFormat : uint8_t { RGB16 = 0, RGB24 = 1, RGBA32 = 2, INVALID=0xFF };

// ASCAL header layout (packed, big-endian u16 fields).
#pragma pack(push,1)
struct FbHeader {
    uint8_t  ty;                // = 0x01 for scaler buffer
    uint8_t  pixel_fmt;         // 0=RGB565,1=RGB888,2=RGBA8888
    uint16_t header_len_be;     // bytes
    uint16_t attributes_be;     // bit 4 triple-buffered; bits 7..5 = "frame counter"
    uint16_t width_be;
    uint16_t height_be;
    uint16_t line_be;           // stride in bytes
    uint16_t out_width_be;
    uint16_t out_height_be;
};
#pragma pack(pop)

// read big-endian 16-bit
static inline uint16_t be16(uint16_t x){ return (uint16_t)((x>>8) | (x<<8)); }

static inline uint8_t frame_counter_from_attr(uint16_t attr_be){
    uint16_t attr = be16(attr_be);
    return (uint8_t)((attr >> 5) & 0x07);
}

static inline bool triple_buffered(uint16_t attr_be){
    return (be16(attr_be) & (1u<<4)) != 0;
}

// Framebuffer type detection: offsets used by MiSTer scaler triple-buffering.
// Small-triple uses 0x0020_0000/0x0040_0000; large-triple uses 0x0080_0000/0x0100_0000.
static inline size_t fb_offset_for_index(bool triple_large, uint8_t index){
    if(index==0) return 0;
    if(triple_large){
        return (index==1) ? 0x00800000u : 0x01000000u;
    } else {
        return (index==1) ? 0x00200000u : 0x00400000u;
    }
}

struct Opts { int step=16; int sleep_us=2500; } opts;

static bool parse_args(int argc, char** argv){
    for(int i=1;i<argc;i++){
        if(!strcmp(argv[i],"--step") && i+1<argc) { opts.step = atoi(argv[++i]); if(opts.step<1) opts.step=1; }
        else if(!strcmp(argv[i],"--sleep-us") && i+1<argc) { opts.sleep_us = atoi(argv[++i]); if(opts.sleep_us<0) opts.sleep_us=0; }
        else { fprintf(stderr,"Unknown option: %s\n", argv[i]); return false; }
    }
    return true;
}

int main(int argc, char** argv){
    if(!parse_args(argc, argv)) return 2;
    signal(SIGINT, sigint); signal(SIGTERM, sigint);

    // Map scaler
    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if(fd<0){ perror("open(/dev/mem)"); return 1; }
    void* map = mmap(nullptr, MAP_LEN, PROT_READ, MAP_SHARED, fd, FB_BASE_ADDRESS);
    if(map==MAP_FAILED){ perror("mmap"); close(fd); return 1; }
    auto* base = (volatile uint8_t*)map;

    // Read primary header
    FbHeader h{};
    memcpy((void*)&h, (const void*)base, sizeof(FbHeader));
    if(h.ty != 0x01){
        fprintf(stderr,"Scaler header not found at 0x%08zx (ty=%u). Is the scaler running?\n",
                FB_BASE_ADDRESS, (unsigned)h.ty);
        munmap(map, MAP_LEN); close(fd); return 3;
    }

    // Resolve fields
    const uint16_t header_len = be16(h.header_len_be);
    const uint16_t width  = be16(h.width_be);
    const uint16_t height = be16(h.height_be);
    const uint16_t line   = be16(h.line_be);
    const ScalerPixelFormat fmt = (ScalerPixelFormat)h.pixel_fmt;
    const bool is_triple = triple_buffered(h.attributes_be);

    // Detect triple-buffer flavor (small vs large) by probing for a valid header at offset
    auto read_header_at = [&](size_t off, FbHeader& out)->bool{
        if(off + sizeof(FbHeader) > MAP_LEN) return false;
        memcpy((void*)&out, (const void*)(base + off), sizeof(FbHeader));
        return out.ty == 0x01;
    };
    bool triple_large=false;
    if(is_triple){
        FbHeader h_small{}, h_large{};
        bool small_ok = read_header_at(0x00200000u, h_small);
        bool large_ok = read_header_at(0x00800000u, h_large);
        triple_large = large_ok && h_large.ty==0x01 && h_large.pixel_fmt<=2;
    }

    fprintf(stdout,
        "MiSTer scaler detected @0x%08zx\n"
        "  fmt=%s  header_len=%u  triple=%s  %ux%u  line=%u\n"
        "  sampling step=%d  sleep=%d us\n",
        FB_BASE_ADDRESS,
        (fmt==RGB24?"RGB24":fmt==RGBA32?"RGBA32":fmt==RGB16?"RGB16":"INVALID"),
        (unsigned)header_len, is_triple? (triple_large?"large":"small"):"no",
        (unsigned)width,(unsigned)height,(unsigned)line,
        opts.step, opts.sleep_us);
    fflush(stdout);

    // frame counter pointers for up to 3 buffers (header byte index 5 in MiSTer helper)
    // In the Rust helper this is "attributes" field; summing 3 counters is used to wait for changes.
    volatile const uint8_t* fc_ptrs[3] = {
        base + 5,
        base + fb_offset_for_index(triple_large,1) + 5,
        base + fb_offset_for_index(triple_large,2) + 5
    };

    // Pixel base pointers (header_len bytes before first pixel)
    auto pixel_base_for = [&](uint8_t idx)->volatile const uint8_t*{
        size_t off = fb_offset_for_index(triple_large, idx);
        return base + off + header_len;
    };

    // Simple frame-change wait using summed counters across all active buffers
    auto read_fc_sum = [&]()->uint8_t{
        uint8_t s=0;
        s += fc_ptrs[0][0];
        if(is_triple) { s += fc_ptrs[1][0]; s += fc_ptrs[2][0]; }
        return s;
    };

    uint32_t last_hash = 0;
    bool first = true;
    uint64_t last_change_ns = now_ns();

    fprintf(stdout,"time(s)  unchanged(s)  avg_rgb  center_rgb  %ux%u\n", width, height);
    fflush(stdout);

    // Helper lambdas to load a pixel at pointer p (format-specific)
    auto load_rgb24 = [](const volatile uint8_t* p, uint8_t& r, uint8_t& g, uint8_t& b){
        r = p[0]; g = p[1]; b = p[2];
    };
    auto load_rgba32 = [](const volatile uint8_t* p, uint8_t& r, uint8_t& g, uint8_t& b){
        r = p[0]; g = p[1]; b = p[2]; /* p[3] alpha ignored */
    };
    auto load_rgb565 = [](const volatile uint8_t* p, uint8_t& r, uint8_t& g, uint8_t& b){
        uint16_t v = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
        r = (uint8_t)(((v >> 11) & 0x1F) * 255 / 31);
        g = (uint8_t)(((v >> 5)  & 0x3F) * 255 / 63);
        b = (uint8_t)(( v        & 0x1F) * 255 / 31);
    };

    while(g_run){
        // Wait briefly for (likely) new frame
        uint8_t last = read_fc_sum();
        for(int i=0;i<4 && g_run;i++){
            if(read_fc_sum() != last) break;
            if(opts.sleep_us>0) usleep(opts.sleep_us);
        }
        if(!g_run) break;

        // sample sparse grid over whichever buffer is "current".
        // We can just read buffer 0; counters change across all and header is updated first.
        const volatile uint8_t* pix0 = pixel_base_for(0);

        const int xs = (opts.step<1)?1:opts.step;
        const int ys = xs;
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
        } else {
            fprintf(stderr,"Unsupported pixel format (%u)\n",(unsigned)fmt);
            break;
        }

        // averages
        double r_avg = n ? (double)rs/n : 0.0;
        double g_avg = n ? (double)gs/n : 0.0;
        double b_avg = n ? (double)bs/n : 0.0;
        unsigned R=(unsigned)r_avg, G=(unsigned)g_avg, B=(unsigned)b_avg;

        // center pixel (using buffer 0)
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

        // change detection by sample hash (cheap) + timer
        uint64_t t_ns = now_ns();
        if(first || hsh != last_hash){
            last_hash = hsh;
            first = false;
            // reset last-change time on visual change
            // (If you prefer using the header checksum alone, use frame_counter_from_attr)
            // But the hash is robust against counters that don't strictly increment.
            static_cast<void>(0);
            // record now
            // (we’ll subtract soon)
        }
        static uint64_t last_change_mark = t_ns;
        if(hsh != last_hash){ last_change_mark = t_ns; } // (kept above; here for clarity)

        double t_s = t_ns / 1e9;
        double unchanged_s = (t_ns - last_change_mark) / 1e9;

        printf("%.3f  %.3f  #%02X%02X%02X  #%02X%02X%02X\n",
               t_s, unchanged_s, R,G,B, Rc,Gc,Bc);
        fflush(stdout);

        if(opts.sleep_us>0) usleep(opts.sleep_us);
    }

    munmap((void*)map, MAP_LEN);
    close(fd);
    return 0;
}
