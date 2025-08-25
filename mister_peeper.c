#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <string.h>
#include <limits.h>

#define MISTER_SCALER_BASEADDR 0x20000000
#define MISTER_SCALER_BUFFERSIZE (2048*3*1024)

// Simple named colors table
typedef struct {
    const char *name;
    uint8_t r, g, b;
} NamedColor;

static NamedColor color_table[] = {
    {"Black",   0,   0,   0},
    {"White", 255, 255, 255},
    {"Red",   255,   0,   0},
    {"Green",   0, 255,   0},
    {"Blue",    0,   0, 255},
    {"Yellow", 255, 255,   0},
    {"Cyan",    0, 255, 255},
    {"Magenta",255,   0, 255},
    {"Gray",  128, 128, 128},
    {"Orange",255, 165,   0},
    {"Purple",128,   0, 128},
    {"Pink",  255, 192, 203}
};
static int color_table_size = sizeof(color_table)/sizeof(color_table[0]);

// Nearest human-readable color name
static const char* nearest_color_name(uint8_t r, uint8_t g, uint8_t b) {
    int best_idx = 0;
    long best_dist = LONG_MAX;
    for (int i=0; i<color_table_size; i++) {
        long dr = r - color_table[i].r;
        long dg = g - color_table[i].g;
        long db = b - color_table[i].b;
        long dist = dr*dr + dg*dg + db*db;
        if (dist < best_dist) {
            best_dist = dist;
            best_idx = i;
        }
    }
    return color_table[best_idx].name;
}

static void read_tmp_file(const char *path, char *out, size_t maxlen) {
    FILE *f = fopen(path, "r");
    if (!f) {
        snprintf(out, maxlen, "Unknown");
        return;
    }
    if (!fgets(out, maxlen, f)) {
        snprintf(out, maxlen, "Unknown");
        fclose(f);
        return;
    }
    out[strcspn(out, "\r\n")] = 0; // strip newline
    fclose(f);
}

int main() {
    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    uint8_t *map = mmap(NULL, MISTER_SCALER_BUFFERSIZE,
                        PROT_READ, MAP_SHARED, fd, MISTER_SCALER_BASEADDR);
    if (map == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }

    long frame_count = 0;
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    int last_cnt = -1;
    double fps = 0.0;

    char rom[256] = "Unknown";
    static int last_len = 0;

    // Static detection
    static uint32_t last_hash = 0;
    static int static_counter = 0;

    while (1) {
        uint8_t *buffer = map;

        if (!(buffer[0] == 1 && buffer[1] == 1)) {
            usleep(2000);
            continue;
        }

        // --- Frame counter (for FPS) ---
        uint8_t flags   = buffer[5];
        int frame_cnt   = (flags >> 5) & 0x07;

        if (last_cnt == -1) {
            last_cnt = frame_cnt;
        } else if (frame_cnt != last_cnt) {
            int delta = (frame_cnt - last_cnt) & 0x07;
            if (delta > 0) frame_count += delta;
            last_cnt = frame_cnt;
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - start.tv_sec) +
                         (now.tv_nsec - start.tv_nsec) / 1e9;

        if (elapsed >= 0.1) {   // update interval
            fps = frame_count / elapsed;
            frame_count = 0;
            start = now;

            // --- Read scaler header ---
            int hdr_offset  = (buffer[3] << 8) | buffer[4];
            int width       = (buffer[6] << 8) | buffer[7];
            int height      = (buffer[8] << 8) | buffer[9];
            int stride      = (buffer[10] << 8) | buffer[11];
            int out_w       = (buffer[12] << 8) | buffer[13];
            int out_h       = (buffer[14] << 8) | buffer[15];
            uint8_t format  = buffer[16];

            // --- Game refresh (1 Hz) ---
            read_tmp_file("/tmp/SAM_Game.txt", rom, sizeof(rom));
            if (strcmp(rom, "Unknown") == 0 || strlen(rom) == 0) {
                read_tmp_file("/tmp/ROM", rom, sizeof(rom));
                if (strcmp(rom, "Unknown") == 0 || strlen(rom) == 0) {
                    read_tmp_file("/tmp/NAME", rom, sizeof(rom));
                }
            }

            // --- Static detection & Dominant color ---
            uint8_t *fb = buffer + hdr_offset;
            uint32_t hash = 0;

            int sample_count = 0;
            long r_sum=0, g_sum=0, b_sum=0;

            for (int y = 0; y < height; y += 16) {
                uint8_t *row = fb + y * stride;
                for (int x = 0; x < width; x += 16) {
                    uint8_t *pix = row + x*3;
                    uint8_t b = pix[0];
                    uint8_t r = pix[1];
                    uint8_t g = pix[2];

                    hash = (hash * 131) + r + (g << 8) + (b << 16);
                    r_sum += r;
                    g_sum += g;
                    b_sum += b;
                    sample_count++;
                }
            }

            if (hash == last_hash) {
                static_counter++;
            } else {
                static_counter = 0;
                last_hash = hash;
            }

            double static_seconds = static_counter * elapsed;  // convert ticks → seconds

            uint8_t dom_r=0, dom_g=0, dom_b=0;
            if (sample_count > 0) {
                dom_r = r_sum / sample_count;
                dom_g = g_sum / sample_count;
                dom_b = b_sum / sample_count;
            }

            char hex_color[16];
            snprintf(hex_color, sizeof(hex_color), "#%02X%02X%02X", dom_r, dom_g, dom_b);
            const char *human_color = nearest_color_name(dom_r, dom_g, dom_b);

            // --- Build output line ---
            char line[1400];
            int len = snprintf(line, sizeof(line),
                "Output=1 | StaticTime=%.1f sec | RGB=%s -> %s | FPS=%6.2f | "
                "Resolution=%4dx%-4d -> %4dx%-4d | Game=%s",
                static_seconds, hex_color, human_color, fps,
                width, height, out_w, out_h, rom);

            printf("\r%s", line);
            if (len < last_len) {
                for (int i = 0; i < last_len - len; i++) putchar(' ');
            }
            last_len = len;
            fflush(stdout);
        }

        usleep(2000);
    }

    munmap(map, MISTER_SCALER_BUFFERSIZE);
    close(fd);
    return 0;
}