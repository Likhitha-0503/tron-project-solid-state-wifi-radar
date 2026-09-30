/**
 ******************************************************************************
 * @file    heatmap.c
 * @brief   WiFi-source heat map renderer (radar fan) - see heatmap.h
 *
 * Picture: sensor at the bottom-centre, field of view +/-60 deg, range
 * HM_RANGE_M. Every 4x4 px cell of the fan gets
 *
 *    heat = A(theta) * R(r)
 *
 *    A(theta): angular term = 50 % beam-pattern interpolation
 *              (sum of squared normalised RSSI x gaussian beam pattern around
 *               +45/+15/-15/-45 deg) + 50 % gaussian around the Kalman theta
 *    R(r)    : radial term  = gaussian around the Kalman distance, sigma grows
 *              with distance (RSSI ranging gets less precise when far)
 *
 * The result is kept with a decay (afterglow), coloured through a 256-entry
 * RGB565 colour map and written straight into the LTDC framebuffer. Only cells
 * whose value changed are redrawn.
 *
 * Font: classic 5x7 ASCII font from the Adafruit GFX library (BSD licence),
 * glyphs 32..126 only.
 ******************************************************************************
 */
#include "heatmap.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ layout */
#define CELL      4
#define ORG_X     60
#define ORG_Y     76
#define GW        170                 /* cells: 170*4 = 680 px  (x 60..739)  */
#define GH        98                  /* cells:  98*4 = 392 px  (y 76..467)  */
#define APEX_X    400
#define APEX_Y    466
#define R_PX      384                 /* radius of the fan in pixels          */
#define FOV_DEG   60
#define NA        (2 * FOV_DEG + 1)   /* angle bins, 1 deg each               */
#define NR        (R_PX / 2 + 1)      /* radius bins, 2 px each               */
#define DEG2RAD   0.01745329252f
#define RAD2DEG   57.29577951f

#define BEAM_SIGMA_DEG   15.0f
#define KF_SIGMA_DEG     12.0f
#define NOT_DRAWN        0xFF         /* heat is limited to 254               */

#define RGB565(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define COL_BLACK   RGB565(0, 0, 0)
#define COL_WHITE   RGB565(255, 255, 255)
#define COL_GRID    RGB565(90, 100, 120)
#define COL_PANEL   RGB565(12, 14, 20)
#define COL_TEXT    RGB565(220, 230, 240)
#define COL_ACCENT  RGB565(255, 190, 40)

static const float BEAM_DEG[4]  = { 45.0f, 15.0f, -15.0f, -45.0f };
static const int8_t ZONE_DEG[5] = { 45, 15, 0, -15, -45 };
static const char  *BEAM_LBL[4] = { "+45", "+15", "-15", "-45" };

/* -------------------------------------------------------------------- data */
static uint16_t *g_fb;
static uint8_t   lut_a[GH][GW];   /* angle bin, 0xFF = cell outside the fan   */
static uint8_t   lut_r[GH][GW];   /* radius bin                               */
static uint8_t   lut_f[GH][GW];   /* 1 = ring / spoke line                    */
static uint8_t   heat [GH][GW];
static uint8_t   drawn[GH][GW];
static float     beam_g[4][NA];
static uint16_t  cmap[256];
static int       mk_valid, mk_x, mk_y;
static int       g_ready;

/* 5x7 font, ASCII 32..126, 5 column bytes per glyph, bit0 = top row */
static const uint8_t font5x7[95 * 5] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x00, 0x00,
    0x00, 0x07, 0x00, 0x07, 0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14,
    0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x23, 0x13, 0x08, 0x64, 0x62,
    0x36, 0x49, 0x56, 0x20, 0x50, 0x00, 0x08, 0x07, 0x03, 0x00,
    0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 0x41, 0x22, 0x1C, 0x00,
    0x2A, 0x1C, 0x7F, 0x1C, 0x2A, 0x08, 0x08, 0x3E, 0x08, 0x08,
    0x00, 0x80, 0x70, 0x30, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x00, 0x00, 0x60, 0x60, 0x00, 0x20, 0x10, 0x08, 0x04, 0x02,
    0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00,
    0x72, 0x49, 0x49, 0x49, 0x46, 0x21, 0x41, 0x49, 0x4D, 0x33,
    0x18, 0x14, 0x12, 0x7F, 0x10, 0x27, 0x45, 0x45, 0x45, 0x39,
    0x3C, 0x4A, 0x49, 0x49, 0x31, 0x41, 0x21, 0x11, 0x09, 0x07,
    0x36, 0x49, 0x49, 0x49, 0x36, 0x46, 0x49, 0x49, 0x29, 0x1E,
    0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x40, 0x34, 0x00, 0x00,
    0x00, 0x08, 0x14, 0x22, 0x41, 0x14, 0x14, 0x14, 0x14, 0x14,
    0x00, 0x41, 0x22, 0x14, 0x08, 0x02, 0x01, 0x59, 0x09, 0x06,
    0x3E, 0x41, 0x5D, 0x59, 0x4E, 0x7C, 0x12, 0x11, 0x12, 0x7C,
    0x7F, 0x49, 0x49, 0x49, 0x36, 0x3E, 0x41, 0x41, 0x41, 0x22,
    0x7F, 0x41, 0x41, 0x41, 0x3E, 0x7F, 0x49, 0x49, 0x49, 0x41,
    0x7F, 0x09, 0x09, 0x09, 0x01, 0x3E, 0x41, 0x41, 0x51, 0x73,
    0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x41, 0x7F, 0x41, 0x00,
    0x20, 0x40, 0x41, 0x3F, 0x01, 0x7F, 0x08, 0x14, 0x22, 0x41,
    0x7F, 0x40, 0x40, 0x40, 0x40, 0x7F, 0x02, 0x1C, 0x02, 0x7F,
    0x7F, 0x04, 0x08, 0x10, 0x7F, 0x3E, 0x41, 0x41, 0x41, 0x3E,
    0x7F, 0x09, 0x09, 0x09, 0x06, 0x3E, 0x41, 0x51, 0x21, 0x5E,
    0x7F, 0x09, 0x19, 0x29, 0x46, 0x26, 0x49, 0x49, 0x49, 0x32,
    0x03, 0x01, 0x7F, 0x01, 0x03, 0x3F, 0x40, 0x40, 0x40, 0x3F,
    0x1F, 0x20, 0x40, 0x20, 0x1F, 0x3F, 0x40, 0x38, 0x40, 0x3F,
    0x63, 0x14, 0x08, 0x14, 0x63, 0x03, 0x04, 0x78, 0x04, 0x03,
    0x61, 0x59, 0x49, 0x4D, 0x43, 0x00, 0x7F, 0x41, 0x41, 0x41,
    0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x41, 0x41, 0x41, 0x7F,
    0x04, 0x02, 0x01, 0x02, 0x04, 0x40, 0x40, 0x40, 0x40, 0x40,
    0x00, 0x03, 0x07, 0x08, 0x00, 0x20, 0x54, 0x54, 0x78, 0x40,
    0x7F, 0x28, 0x44, 0x44, 0x38, 0x38, 0x44, 0x44, 0x44, 0x28,
    0x38, 0x44, 0x44, 0x28, 0x7F, 0x38, 0x54, 0x54, 0x54, 0x18,
    0x00, 0x08, 0x7E, 0x09, 0x02, 0x18, 0xA4, 0xA4, 0x9C, 0x78,
    0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x44, 0x7D, 0x40, 0x00,
    0x20, 0x40, 0x40, 0x3D, 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00,
    0x00, 0x41, 0x7F, 0x40, 0x00, 0x7C, 0x04, 0x78, 0x04, 0x78,
    0x7C, 0x08, 0x04, 0x04, 0x78, 0x38, 0x44, 0x44, 0x44, 0x38,
    0xFC, 0x18, 0x24, 0x24, 0x18, 0x18, 0x24, 0x24, 0x18, 0xFC,
    0x7C, 0x08, 0x04, 0x04, 0x08, 0x48, 0x54, 0x54, 0x54, 0x24,
    0x04, 0x04, 0x3F, 0x44, 0x24, 0x3C, 0x40, 0x40, 0x20, 0x7C,
    0x1C, 0x20, 0x40, 0x20, 0x1C, 0x3C, 0x40, 0x30, 0x40, 0x3C,
    0x44, 0x28, 0x10, 0x28, 0x44, 0x4C, 0x90, 0x90, 0x90, 0x7C,
    0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, 0x08, 0x36, 0x41, 0x00,
    0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 0x41, 0x36, 0x08, 0x00,
    0x02, 0x01, 0x02, 0x04, 0x02,
};

__attribute__((weak)) void hm_cache_clean(void *addr, uint32_t size)
{
    (void)addr; (void)size;
}

/* -------------------------------------------------------------- primitives */
static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static void fill_rect(int x, int y, int w, int h, uint16_t c)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > HM_SCREEN_W) w = HM_SCREEN_W - x;
    if (y + h > HM_SCREEN_H) h = HM_SCREEN_H - y;
    if (w <= 0 || h <= 0) return;
    for (int yy = y; yy < y + h; yy++) {
        uint16_t *p = &g_fb[yy * HM_SCREEN_W + x];
        for (int xx = 0; xx < w; xx++) p[xx] = c;
    }
}

static void put_px(int x, int y, uint16_t c)
{
    if ((unsigned)x < HM_SCREEN_W && (unsigned)y < HM_SCREEN_H)
        g_fb[y * HM_SCREEN_W + x] = c;
}

static void draw_char(int x, int y, char ch, int sc, uint16_t fg, uint16_t bg)
{
    if (ch < 32 || ch > 126) ch = '?';
    const uint8_t *g = &font5x7[(ch - 32) * 5];
    for (int col = 0; col < 6; col++) {
        uint8_t bits = (col < 5) ? g[col] : 0;
        for (int row = 0; row < 8; row++) {
            int on = (row < 7) && ((bits >> row) & 1);
            fill_rect(x + col * sc, y + row * sc, sc, sc, on ? fg : bg);
        }
    }
}

static void draw_text(int x, int y, const char *s, int sc, uint16_t fg, uint16_t bg)
{
    for (; *s; s++, x += 6 * sc) draw_char(x, y, *s, sc, fg, bg);
}

static char *cat(char *p, const char *s)
{
    while (*s) *p++ = *s++;
    return p;
}

static char *cat_int(char *p, int v)
{
    char t[12]; int n = 0;
    if (v < 0) { *p++ = '-'; v = -v; }
    if (v == 0) t[n++] = '0';
    while (v > 0) { t[n++] = (char)('0' + v % 10); v /= 10; }
    while (n > 0) *p++ = t[--n];
    return p;
}

static char *cat_sint(char *p, int v)          /* always with sign */
{
    if (v >= 0) *p++ = '+';
    return cat_int(p, v);
}

/* pad with spaces to exactly n characters and terminate */
static void pad(char *start, char *end, int n)
{
    while ((end - start) < n) *end++ = ' ';
    start[n] = '\0';
}

/* ---------------------------------------------------------------- colormap */
static void build_cmap(void)
{
    static const struct { float t; uint8_t r, g, b; } s[] = {
        { 0.00f,   8,  10,  48 }, { 0.15f,   0,  60, 190 },
        { 0.35f,   0, 190, 210 }, { 0.55f,  60, 220,  70 },
        { 0.75f, 250, 225,   0 }, { 0.90f, 255,  80,   0 },
        { 1.00f, 255, 255, 255 }
    };
    for (int i = 0; i < 256; i++) {
        float t = (float)i / 255.0f;
        int k = 0;
        while (k < 5 && t > s[k + 1].t) k++;
        float u = (t - s[k].t) / (s[k + 1].t - s[k].t);
        int r = (int)(s[k].r + u * (s[k + 1].r - s[k].r));
        int g = (int)(s[k].g + u * (s[k + 1].g - s[k].g));
        int b = (int)(s[k].b + u * (s[k + 1].b - s[k].b));
        cmap[i] = RGB565(r, g, b);
    }
}

/* --------------------------------------------------------------- tables    */
static void build_luts(void)
{
    const float px_per_m = (float)R_PX / HM_RANGE_M;
    const float ring_px  = 2.0f * px_per_m;         /* ring every 2 m */

    for (int b = 0; b < 4; b++)
        for (int a = 0; a < NA; a++) {
            float d = (float)(a - FOV_DEG) - BEAM_DEG[b];
            beam_g[b][a] = expf(-(d * d) / (2.0f * BEAM_SIGMA_DEG * BEAM_SIGMA_DEG));
        }

    for (int gy = 0; gy < GH; gy++)
        for (int gx = 0; gx < GW; gx++) {
            float cx = ORG_X + gx * CELL + CELL * 0.5f;
            float cy = ORG_Y + gy * CELL + CELL * 0.5f;
            float dx = cx - APEX_X, dy = APEX_Y - cy;
            float r  = sqrtf(dx * dx + dy * dy);
            float ang = atan2f(dx, dy) * RAD2DEG;    /* 0 = straight ahead */
            if (HM_MIRROR) ang = -ang;

            lut_a[gy][gx] = 0xFF; lut_r[gy][gx] = 0; lut_f[gy][gx] = 0;
            heat[gy][gx]  = 0;    drawn[gy][gx] = NOT_DRAWN;

            float lim = FOV_DEG + RAD2DEG * 2.0f / (r > 1.0f ? r : 1.0f);
            if (dy <= 0.0f || r > R_PX + 2.0f || fabsf(ang) > lim) continue;

            int ai = (int)floorf(ang + FOV_DEG + 0.5f);
            int ri = (int)(r * 0.5f);
            if (ai < 0) ai = 0;
            if (ai > NA - 1) ai = NA - 1;
            if (ri > NR - 1) ri = NR - 1;
            lut_a[gy][gx] = (uint8_t)ai;
            lut_r[gy][gx] = (uint8_t)ri;

            float k = roundf(r / ring_px);
            if (k >= 1.0f && fabsf(r - k * ring_px) <= 2.0f) lut_f[gy][gx] = 1;
            float m = roundf(ang / 30.0f) * 30.0f;
            if (r > 10.0f && fabsf(r * sinf((ang - m) * DEG2RAD)) <= 1.5f) lut_f[gy][gx] = 1;
        }
}

/* ------------------------------------------------------------ screen parts */
static void draw_static(void)
{
    fill_rect(0, 0, HM_SCREEN_W, HM_SCREEN_H, COL_BLACK);
    draw_text(10, 6, "WIFI SOURCE HEAT MAP", 3, COL_WHITE, COL_BLACK);

    /* legend */
    for (int x = 0; x < 260; x++)
        fill_rect(520 + x, 6, 1, 12, cmap[x * 255 / 259]);
    draw_text(520, 20, "WEAK", 1, COL_TEXT, COL_BLACK);
    draw_text(780 - 6 * 6, 20, "STRONG", 1, COL_TEXT, COL_BLACK);

    /* ring labels, just outside the fan on the right-hand side */
    const float ring_px = 2.0f * (float)R_PX / HM_RANGE_M;
    for (int k = 1; (k * 2.0f) <= HM_RANGE_M + 0.01f; k++) {
        char t[8]; char *p = cat_int(t, k * 2); p = cat(p, "M"); *p = '\0';
        float r = k * ring_px;
        draw_text((int)(APEX_X + r * 0.9135f), (int)(APEX_Y - r * 0.4067f), t, 2, COL_TEXT, COL_BLACK);
    }
    draw_text(8, 362, "BEAM RSSI", 2, COL_ACCENT, COL_BLACK);
}

static void draw_cells(void)
{
    for (int gy = 0; gy < GH; gy++)
        for (int gx = 0; gx < GW; gx++) {
            if (lut_a[gy][gx] == 0xFF) continue;
            uint8_t v = heat[gy][gx];
            if (drawn[gy][gx] == v) continue;
            drawn[gy][gx] = v;
            uint16_t c = (lut_f[gy][gx] && v < 24) ? COL_GRID : cmap[v];
            fill_rect(ORG_X + gx * CELL, ORG_Y + gy * CELL, CELL, CELL, c);
        }
}

static void mark_dirty(int x0, int y0, int x1, int y1)
{
    int gx0 = (x0 - ORG_X) / CELL, gx1 = (x1 - ORG_X) / CELL;
    int gy0 = (y0 - ORG_Y) / CELL, gy1 = (y1 - ORG_Y) / CELL;
    if (gx0 < 0) gx0 = 0;
    if (gx1 > GW - 1) gx1 = GW - 1;
    if (gy0 < 0) gy0 = 0;
    if (gy1 > GH - 1) gy1 = GH - 1;
    for (int gy = gy0; gy <= gy1; gy++)
        for (int gx = gx0; gx <= gx1; gx++) drawn[gy][gx] = NOT_DRAWN;
}

static void draw_marker(int cx, int cy)
{
    for (int i = -11; i <= 11; i++) {
        if (i > -4 && i < 4) continue;               /* gap in the middle */
        put_px(cx + i, cy, COL_WHITE);  put_px(cx, cy + i, COL_WHITE);
    }
    for (int a = 0; a < 360; a += 10) {
        float rad = a * DEG2RAD;
        put_px(cx + (int)(7.5f * cosf(rad)), cy + (int)(7.5f * sinf(rad)), COL_WHITE);
    }
}

static void draw_info(const hm_frame_t *f, int have)
{
    char line[64], *p;

    if (have) {
        int zone = f->zone > 4 ? 4 : f->zone;
        int d100 = (int)(f->dist_m * 100.0f + 0.5f);
        p = cat(line, "ZONE ");           p = cat_int(p, zone + 1);
        p = cat(p, "  ");                 p = cat_sint(p, ZONE_DEG[zone]);
        p = cat(p, " DEG   DIST ");       p = cat_int(p, d100 / 100);
        *p++ = '.';                       *p++ = (char)('0' + (d100 / 10) % 10);
        *p++ = (char)('0' + d100 % 10);   p = cat(p, " M");
    } else {
        p = cat(line, "NO SIGNAL");
    }
    pad(line, p, 44);
    draw_text(10, 32, line, 2, COL_ACCENT, COL_BLACK);

    if (have) {
        int peak = -128;
        for (int b = 0; b < 4; b++) if (f->beam_rssi[b] > peak) peak = f->beam_rssi[b];
        p = cat(line, "THETA ");          p = cat_sint(p, (int)(f->theta_deg + (f->theta_deg < 0 ? -0.5f : 0.5f)));
        p = cat(p, " DEG   PEAK ");       p = cat_int(p, peak);
        p = cat(p, " DBM");
    } else {
        p = cat(line, "WAITING FOR BEAMS");
    }
    pad(line, p, 44);
    draw_text(10, 52, line, 2, COL_TEXT, COL_BLACK);

    /* per-beam bars, bottom-left */
    for (int b = 0; b < 4; b++) {
        int y = 384 + b * 22;
        float n = have ? clampf(f->beam_norm[b], 0.0f, 1.0f) : 0.0f;
        int w = (int)(n * 100.0f);
        draw_text(8, y, BEAM_LBL[b], 2, COL_TEXT, COL_BLACK);
        fill_rect(48, y, 100, 14, COL_PANEL);
        fill_rect(48, y, w, 14, cmap[(int)(n * 254.0f)]);
        p = have ? cat_int(line, f->beam_rssi[b]) : cat(line, "--");
        pad(line, p, 4);
        draw_text(154, y, line, 2, COL_TEXT, COL_BLACK);
    }
}

/* -------------------------------------------------------------- public API */
void hm_init(uint16_t *framebuffer)
{
    g_fb = framebuffer;
    build_cmap();
    build_luts();
    mk_valid = 0;
    draw_static();
    g_ready = 1;
    hm_update(NULL);
}

void hm_update(const hm_frame_t *f)
{
    static float A[NA], araw[NA], Rr[NR];
    if (!g_ready) return;
    const int have = (f != NULL) && f->valid;
    const float px_per_m = (float)R_PX / HM_RANGE_M;
    float dist = 0.0f, theta = 0.0f;

    if (have) {
        float w[4], maxn = 0.0f, amax = 1e-6f;
        for (int b = 0; b < 4; b++) {
            float n = clampf(f->beam_norm[b], 0.0f, 1.0f);
            w[b] = n * n;
            if (n > maxn) maxn = n;
        }
        theta = clampf(f->theta_deg, -(float)FOV_DEG, (float)FOV_DEG);
        dist  = clampf(f->dist_m, 0.1f, HM_RANGE_M);
        for (int a = 0; a < NA; a++) {
            float s = 0.0f;
            for (int b = 0; b < 4; b++) s += w[b] * beam_g[b][a];
            araw[a] = s;
            if (s > amax) amax = s;
        }
        const float conf = 0.35f + 0.65f * maxn;      /* weak signal = dim */
        for (int a = 0; a < NA; a++) {
            float dk = (float)(a - FOV_DEG) - theta;
            float kg = expf(-(dk * dk) / (2.0f * KF_SIGMA_DEG * KF_SIGMA_DEG));
            A[a] = clampf(conf * (0.5f * araw[a] / amax + 0.5f * kg), 0.0f, 1.0f);
        }
        const float sigma = 0.35f + 0.10f * dist;
        for (int i = 0; i < NR; i++) {
            float e = (((float)(i * 2 + 1) / px_per_m) - dist) / sigma;
            Rr[i] = expf(-0.5f * e * e);
        }
    }

    /* 1. update heat (persistence) */
    for (int gy = 0; gy < GH; gy++)
        for (int gx = 0; gx < GW; gx++) {
            uint8_t a = lut_a[gy][gx];
            if (a == 0xFF) continue;
            uint8_t old = (uint8_t)(((unsigned)heat[gy][gx] * HM_DECAY_Q8) >> 8);
            uint8_t nw  = have ? (uint8_t)(A[a] * Rr[lut_r[gy][gx]] * 254.0f) : 0;
            heat[gy][gx] = (nw > old) ? nw : old;
        }

    /* 2. erase old marker, redraw changed cells, draw new marker */
    if (mk_valid) mark_dirty(mk_x - 12, mk_y - 12, mk_x + 12, mk_y + 12);
    draw_cells();
    if (have) {
        float rpx = dist * px_per_m, rad = theta * DEG2RAD;
        float sgn = HM_MIRROR ? -1.0f : 1.0f;
        mk_x = (int)(APEX_X + sgn * rpx * sinf(rad));
        mk_y = (int)(APEX_Y - rpx * cosf(rad));
        mk_valid = 1;
        draw_marker(mk_x, mk_y);
    } else {
        mk_valid = 0;
    }

    /* 3. text + bars */
    draw_info(f, have);

    hm_cache_clean(g_fb, (uint32_t)HM_SCREEN_W * HM_SCREEN_H * 2u);
}

void hm_test_pattern(uint16_t *fb)
{
    static const uint16_t bars[8] = {
        RGB565(255,255,255), RGB565(255,255,0), RGB565(0,255,255), RGB565(0,255,0),
        RGB565(255,0,255),   RGB565(255,0,0),   RGB565(0,0,255),   RGB565(0,0,0)
    };
    g_fb = fb;
    if (cmap[255] == 0) build_cmap();
    for (int i = 0; i < 8; i++) fill_rect(i * 100, 0, 100, 400, bars[i]);
    for (int x = 0; x < HM_SCREEN_W; x++) fill_rect(x, 400, 1, 80, cmap[x * 255 / (HM_SCREEN_W - 1)]);
    fill_rect(0, 0, HM_SCREEN_W, 3, COL_WHITE);           /* border */
    fill_rect(0, HM_SCREEN_H - 3, HM_SCREEN_W, 3, COL_WHITE);
    fill_rect(0, 0, 3, HM_SCREEN_H, COL_WHITE);
    fill_rect(HM_SCREEN_W - 3, 0, 3, HM_SCREEN_H, COL_WHITE);
    draw_text(250, 180, "LCD TEST 800X480", 3, COL_BLACK, COL_WHITE);
    hm_cache_clean(g_fb, (uint32_t)HM_SCREEN_W * HM_SCREEN_H * 2u);
}
