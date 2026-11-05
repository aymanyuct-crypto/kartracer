/*
 * TURBO LOOP - an original 3D kart racer for PSP (PSPSDK, C, sceGu/sceGum)
 *
 * Controls
 *   Analog stick / D-pad  steer
 *   X                     accelerate
 *   Square                brake / reverse
 *   R                     drift (hold while steering, release for a boost)
 *   Triangle              use item (boost mushroom from item boxes)
 *   Start                 restart race
 *
 * Everything (track, karts, trees) is generated procedurally, so there are
 * no external assets to load. Vertex data is static for speed.
 */
#include <pspkernel.h>
#include <pspdisplay.h>
#include <pspdebug.h>
#include <pspctrl.h>
#include <pspgu.h>
#include <pspgum.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

PSP_MODULE_INFO("TurboLoop", 0, 1, 0);
PSP_MAIN_THREAD_ATTR(THREAD_ATTR_USER | THREAD_ATTR_VFPU);

/* ---------- exit callback ---------- */
static int exit_cb(int a1, int a2, void *c) { sceKernelExitGame(); return 0; }
static int cb_thread(SceSize args, void *argp) {
    int id = sceKernelCreateCallback("exit_cb", exit_cb, NULL);
    sceKernelRegisterExitCallback(id);
    sceKernelSleepThreadCB();
    return 0;
}
static void setup_callbacks(void) {
    int t = sceKernelCreateThread("cb", cb_thread, 0x11, 0xFA0, 0, 0);
    if (t >= 0) sceKernelStartThread(t, 0, 0);
}

/* ---------- constants ---------- */
#define BUF_W 512
#define SCR_W 480
#define SCR_H 272
#define PI_F  3.14159265f

#define NCP    12            /* control points */
#define SUB    16            /* samples per control segment */
#define NS     (NCP * SUB)   /* centerline samples */
#define SCALE  2.5f
#define HALFW  14.0f         /* road half width */
#define CURB   2.0f
#define NKARTS 4
#define NLAPS  3
#define NBOX   8
#define NTREE  70

#define MAXSPD 0.85f
#define ACCEL  0.015f
#define TURN   0.040f

#define RGB(r,g,b) (0xff000000u | ((b)<<16) | ((g)<<8) | (r))

typedef struct { float x, z; } Vec2;
typedef struct { unsigned int c; float x, y, z; } CVert;
typedef struct { float x, y, z; } PVert;

typedef struct {
    float x, z, h, speed;
    int idx, total;
    int boost, drift, charge, item, finished, finishFrame;
    float driftDir, skill;
    int isAI;
    unsigned int color;
} Kart;

/* ---------- track data ---------- */
static const Vec2 cp[NCP] = {
    {0,-120},{80,-110},{140,-60},{150,20},{100,80},{40,60},
    {0,100},{-60,130},{-130,90},{-150,10},{-120,-70},{-60,-110}
};
static float cx[NS], cz[NS], nx[NS], nz[NS];

static CVert roadV[2*(NS+1)] __attribute__((aligned(16)));
static CVert curbLV[2*(NS+1)] __attribute__((aligned(16)));
static CVert curbRV[2*(NS+1)] __attribute__((aligned(16)));
static CVert groundV[4] __attribute__((aligned(16)));
static PVert cube[36] __attribute__((aligned(16)));
static PVert pyramid[12] __attribute__((aligned(16)));
static Vec2 trees[NTREE];

static Kart karts[NKARTS];
static Vec2 boxPos[NBOX];
static int boxTimer[NBOX];

static unsigned int __attribute__((aligned(16))) dlist[262144];
static int frameCount;
static int countdown;

/* ---------- helpers ---------- */
static float wrapPi(float a) {
    while (a >  PI_F) a -= 2*PI_F;
    while (a < -PI_F) a += 2*PI_F;
    return a;
}
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void buildMeshes(void) {
    /* unit cube centered on origin */
    static const float c[8][3] = {
        {-.5f,-.5f,-.5f},{.5f,-.5f,-.5f},{.5f,.5f,-.5f},{-.5f,.5f,-.5f},
        {-.5f,-.5f, .5f},{.5f,-.5f, .5f},{.5f,.5f, .5f},{-.5f,.5f, .5f}};
    static const int f[6][4] = {{0,1,2,3},{5,4,7,6},{4,0,3,7},{1,5,6,2},{3,2,6,7},{4,5,1,0}};
    int n = 0;
    for (int i = 0; i < 6; i++) {
        int q[6] = {f[i][0],f[i][1],f[i][2],f[i][0],f[i][2],f[i][3]};
        for (int j = 0; j < 6; j++) {
            cube[n].x = c[q[j]][0]; cube[n].y = c[q[j]][1]; cube[n].z = c[q[j]][2]; n++;
        }
    }
    /* 4-sided pyramid, base at y=0, apex at y=1, width 1 */
    static const float b[4][2] = {{-.5f,-.5f},{.5f,-.5f},{.5f,.5f},{-.5f,.5f}};
    n = 0;
    for (int i = 0; i < 4; i++) {
        int j = (i+1)%4;
        pyramid[n++] = (PVert){b[i][0],0,b[i][1]};
        pyramid[n++] = (PVert){b[j][0],0,b[j][1]};
        pyramid[n++] = (PVert){0,1,0};
    }
}

static void buildTrack(void) {
    int n = 0;
    for (int s = 0; s < NCP; s++) {
        Vec2 p0 = cp[(s+NCP-1)%NCP], p1 = cp[s], p2 = cp[(s+1)%NCP], p3 = cp[(s+2)%NCP];
        for (int j = 0; j < SUB; j++, n++) {
            float t = j/(float)SUB, t2 = t*t, t3 = t2*t;
            cx[n] = SCALE*0.5f*(2*p1.x + (-p0.x+p2.x)*t + (2*p0.x-5*p1.x+4*p2.x-p3.x)*t2 + (-p0.x+3*p1.x-3*p2.x+p3.x)*t3);
            cz[n] = SCALE*0.5f*(2*p1.z + (-p0.z+p2.z)*t + (2*p0.z-5*p1.z+4*p2.z-p3.z)*t2 + (-p0.z+3*p1.z-3*p2.z+p3.z)*t3);
        }
    }
    for (int i = 0; i < NS; i++) {
        float tx = cx[(i+1)%NS] - cx[(i+NS-1)%NS];
        float tz = cz[(i+1)%NS] - cz[(i+NS-1)%NS];
        float l = sqrtf(tx*tx + tz*tz);
        nx[i] = tz/l; nz[i] = -tx/l;
    }
    for (int k = 0; k <= NS; k++) {
        int i = k % NS;
        unsigned int road = ((k/3)&1) ? RGB(95,95,100) : RGB(110,110,115);
        unsigned int curb = ((k/2)&1) ? RGB(220,30,30) : RGB(245,245,245);
        float ex = nx[i], ez = nz[i];
        roadV[2*k]   = (CVert){road, cx[i]+ex*HALFW, 0.15f, cz[i]+ez*HALFW};
        roadV[2*k+1] = (CVert){road, cx[i]-ex*HALFW, 0.15f, cz[i]-ez*HALFW};
        curbLV[2*k]   = (CVert){curb, cx[i]+ex*HALFW,       0.2f, cz[i]+ez*HALFW};
        curbLV[2*k+1] = (CVert){curb, cx[i]+ex*(HALFW+CURB),0.2f, cz[i]+ez*(HALFW+CURB)};
        curbRV[2*k]   = (CVert){curb, cx[i]-ex*HALFW,       0.2f, cz[i]-ez*HALFW};
        curbRV[2*k+1] = (CVert){curb, cx[i]-ex*(HALFW+CURB),0.2f, cz[i]-ez*(HALFW+CURB)};
    }
    unsigned int g = RGB(60,150,60);
    groundV[0] = (CVert){g,-2500,0,-2500};
    groundV[1] = (CVert){g, 2500,0,-2500};
    groundV[2] = (CVert){g,-2500,0, 2500};
    groundV[3] = (CVert){g, 2500,0, 2500};

    for (int i = 0; i < NTREE; i++) {
        int idx = (i * NS / NTREE) % NS;
        float side = (i & 1) ? 1.f : -1.f;
        float off = HALFW + 10.f + (float)((i*7) % 14);
        trees[i].x = cx[idx] + nx[idx]*off*side;
        trees[i].z = cz[idx] + nz[idx]*off*side;
    }
    for (int i = 0; i < NBOX; i++) {
        int idx = (i * NS / NBOX + 6) % NS;
        float off = ((i%3)-1) * 6.f;
        boxPos[i].x = cx[idx] + nx[idx]*off;
        boxPos[i].z = cz[idx] + nz[idx]*off;
        boxTimer[i] = 0;
    }
    sceKernelDcacheWritebackAll();  /* GU reads straight from RAM */
}

/* ---------- race setup ---------- */
static void resetRace(void) {
    static const unsigned int cols[NKARTS] = {RGB(230,40,40), RGB(40,90,230), RGB(240,200,30), RGB(150,50,200)};
    static const float skills[NKARTS] = {1.0f, 0.88f, 0.84f, 0.80f};
    for (int k = 0; k < NKARTS; k++) {
        int back = 3 + 3*k;
        int idx = (NS - back) % NS;
        float side = (k & 1) ? 4.f : -4.f;
        Kart *a = &karts[k];
        memset(a, 0, sizeof(*a));
        a->idx = idx; a->total = -back;
        a->x = cx[idx] + nx[idx]*side;
        a->z = cz[idx] + nz[idx]*side;
        a->h = atan2f(cx[(idx+1)%NS]-cx[idx], cz[(idx+1)%NS]-cz[idx]);
        a->isAI = (k != 0);
        a->skill = skills[k];
        a->color = cols[k];
    }
    for (int i = 0; i < NBOX; i++) boxTimer[i] = 0;
    countdown = 240;
}

/* ---------- kart physics (shared by player and AI) ---------- */
static void updateKart(Kart *k, float steer, int accel, int brake, int driftBtn, int use) {
    /* find nearest centerline sample near previous one */
    int best = k->idx; float bd = 1e9f;
    for (int o = -12; o <= 12; o++) {
        int i = (k->idx + o + NS) % NS;
        float dx = k->x - cx[i], dz = k->z - cz[i];
        float d = dx*dx + dz*dz;
        if (d < bd) { bd = d; best = i; }
    }
    int delta = best - k->idx;
    if (delta < -NS/2) delta += NS;
    if (delta >  NS/2) delta -= NS;
    k->total += delta;
    k->idx = best;
    float dist = sqrtf(bd);

    float cap = MAXSPD * k->skill;
    if (k->boost > 0) { cap = MAXSPD * 1.5f; k->boost--; }
    else if (dist > HALFW + CURB) cap = MAXSPD * 0.4f;   /* grass slows you */

    if (use && k->item > 0) { k->boost = 70; k->item = 0; }

    if (accel) k->speed += ACCEL * (k->boost > 0 ? 2.f : 1.f);
    else       k->speed *= 0.988f;
    if (brake) k->speed -= 0.03f;
    if (k->speed < -0.25f) k->speed = -0.25f;
    if (k->speed > cap) { k->speed -= 0.02f; if (k->speed < cap) k->speed = cap; }

    float sp = fabsf(k->speed);
    float dir = k->speed >= 0 ? 1.f : -1.f;
    float grip = clampf(sp / 0.4f, 0.f, 1.f);

    /* drifting: lock direction, build charge, release for boost */
    if (driftBtn && sp > 0.45f && !k->drift && fabsf(steer) > 0.3f) {
        k->drift = 1; k->driftDir = steer > 0 ? 1.f : -1.f; k->charge = 0;
    }
    if (k->drift) {
        if (!driftBtn || sp < 0.3f) {
            if (k->charge > 90) k->boost = 80;
            else if (k->charge > 45) k->boost = 45;
            k->drift = 0; k->charge = 0;
        } else {
            k->charge++;
            steer = k->driftDir * (0.7f + 0.3f * steer * k->driftDir);
        }
    }
    /* positive steer = right = heading decreases (view is right-handed) */
    k->h -= steer * TURN * grip * dir * (k->drift ? 1.3f : 1.f);
    k->h = wrapPi(k->h);

    k->x += sinf(k->h) * k->speed;
    k->z += cosf(k->h) * k->speed;

    /* invisible walls well off the road */
    float lim = HALFW + CURB + 12.f;
    if (dist > lim) {
        float s = lim / dist;
        k->x = cx[best] + (k->x - cx[best]) * s;
        k->z = cz[best] + (k->z - cz[best]) * s;
        k->speed *= 0.9f;
    }

    if (!k->finished && k->total >= NLAPS * NS) { k->finished = 1; k->finishFrame = frameCount; }
}

static void aiControl(Kart *k, int playerTotal) {
    int ti = (k->idx + 8 + (int)(k->speed * 6.f)) % NS;
    float want = atan2f(cx[ti]-k->x, cz[ti]-k->z);
    float diff = wrapPi(want - k->h);
    float steer = -clampf(diff * 2.f, -1.f, 1.f);
    int accel = fabsf(diff) < 0.7f;
    /* mild rubber-banding keeps the race close */
    float rb = clampf((playerTotal - k->total) * 0.0004f, -0.06f, 0.06f);
    float base = k->skill;
    k->skill = base + rb;
    int use = (k->item > 0) && ((rand() & 63) == 0);
    updateKart(k, steer, accel, 0, 0, use);
    k->skill = base;
}

static void collideKarts(void) {
    for (int a = 0; a < NKARTS; a++)
        for (int b = a+1; b < NKARTS; b++) {
            float dx = karts[b].x - karts[a].x, dz = karts[b].z - karts[a].z;
            float d2 = dx*dx + dz*dz, r = 3.0f;
            if (d2 < r*r && d2 > 0.0001f) {
                float d = sqrtf(d2), push = (r - d) * 0.5f;
                dx /= d; dz /= d;
                karts[a].x -= dx*push; karts[a].z -= dz*push;
                karts[b].x += dx*push; karts[b].z += dz*push;
                karts[a].speed *= 0.97f; karts[b].speed *= 0.97f;
            }
        }
}

static void updateItemBoxes(void) {
    for (int i = 0; i < NBOX; i++) {
        if (boxTimer[i] > 0) { boxTimer[i]--; continue; }
        for (int k = 0; k < NKARTS; k++) {
            float dx = karts[k].x - boxPos[i].x, dz = karts[k].z - boxPos[i].z;
            if (dx*dx + dz*dz < 16.f) {
                if (karts[k].item == 0) karts[k].item = 1;
                boxTimer[i] = 300;
                break;
            }
        }
    }
}

static int placeOf(int k) {
    int p = 1;
    for (int i = 0; i < NKARTS; i++) {
        if (i == k) continue;
        if (karts[i].finished && !karts[k].finished) p++;
        else if (karts[i].finished && karts[k].finished) { if (karts[i].finishFrame < karts[k].finishFrame) p++; }
        else if (!karts[i].finished && !karts[k].finished && karts[i].total > karts[k].total) p++;
    }
    return p;
}

/* ---------- rendering ---------- */
static void box(float x, float y, float z, float sx, float sy, float sz, unsigned int col) {
    ScePspFVector3 t = {x,y,z}, s = {sx,sy,sz};
    sceGumPushMatrix();
    sceGumTranslate(&t);
    sceGumScale(&s);
    sceGuColor(col);
    sceGumDrawArray(GU_TRIANGLES, GU_VERTEX_32BITF|GU_TRANSFORM_3D, 36, 0, cube);
    sceGumPopMatrix();
}

static void drawKart(const Kart *k) {
    ScePspFVector3 p = {k->x, 0, k->z};
    sceGumPushMatrix();
    sceGumTranslate(&p);
    sceGumRotateY(k->h);
    box(0, 0.9f, 0,     1.8f, 0.7f, 3.2f, k->color);
    box(0, 1.7f,-0.3f,  0.9f, 0.9f, 0.9f, RGB(240,200,160));       /* driver */
    box(0, 2.3f,-0.3f,  1.0f, 0.3f, 1.0f, k->color);                 /* cap */
    box(0, 1.5f,-1.7f,  2.0f, 0.2f, 0.4f, k->color);                 /* spoiler */
    box(-1.1f,0.5f, 1.0f, 0.4f,1.0f,1.0f, RGB(25,25,25));
    box( 1.1f,0.5f, 1.0f, 0.4f,1.0f,1.0f, RGB(25,25,25));
    box(-1.1f,0.5f,-1.1f, 0.4f,1.0f,1.0f, RGB(25,25,25));
    box( 1.1f,0.5f,-1.1f, 0.4f,1.0f,1.0f, RGB(25,25,25));
    if (k->drift) {   /* spark color shows drift charge */
        unsigned int c = k->charge > 90 ? RGB(255,120,20) : (k->charge > 45 ? RGB(80,200,255) : RGB(255,255,255));
        box(-0.9f,0.3f,-2.0f,0.3f,0.3f,0.3f,c);
        box( 0.9f,0.3f,-2.0f,0.3f,0.3f,0.3f,c);
    }
    if (k->boost > 0) box(0,0.7f,-2.3f,0.8f,0.5f,1.2f,RGB(255,160,30));
    sceGumPopMatrix();
}

static void drawWorld(float camx, float camz) {
    const unsigned int cfmt = GU_COLOR_8888|GU_VERTEX_32BITF|GU_TRANSFORM_3D;
    sceGumDrawArray(GU_TRIANGLE_STRIP, cfmt, 4, 0, groundV);
    sceGumDrawArray(GU_TRIANGLE_STRIP, cfmt, 2*(NS+1), 0, roadV);
    sceGumDrawArray(GU_TRIANGLE_STRIP, cfmt, 2*(NS+1), 0, curbLV);
    sceGumDrawArray(GU_TRIANGLE_STRIP, cfmt, 2*(NS+1), 0, curbRV);

    /* start / finish line */
    float yaw = atan2f(cx[1]-cx[NS-1], cz[1]-cz[NS-1]);
    ScePspFVector3 sp = {cx[0], 0.3f, cz[0]};
    sceGumPushMatrix(); sceGumTranslate(&sp); sceGumRotateY(yaw);
    for (int i = 0; i < 14; i++) {
        box(-HALFW + 1.f + i*2.f, 0, 0.5f, 2.f, 0.05f, 1.f, (i&1) ? RGB(20,20,20) : RGB(250,250,250));
        box(-HALFW + 1.f + i*2.f, 0,-0.5f, 2.f, 0.05f, 1.f, (i&1) ? RGB(250,250,250) : RGB(20,20,20));
    }
    sceGumPopMatrix();

    /* trees (distance culled) */
    for (int i = 0; i < NTREE; i++) {
        float dx = trees[i].x - camx, dz = trees[i].z - camz;
        if (dx*dx + dz*dz > 300.f*300.f) continue;
        ScePspFVector3 t = {trees[i].x, 0, trees[i].z};
        ScePspFVector3 s = {7.f, 14.f, 7.f};
        sceGumPushMatrix(); sceGumTranslate(&t);
        box(0, 2.f, 0, 1.6f, 4.f, 1.6f, RGB(110,70,30));
        ScePspFVector3 t2 = {0, 3.f, 0};
        sceGumTranslate(&t2); sceGumScale(&s);
        sceGuColor(RGB(20,110,30));
        sceGumDrawArray(GU_TRIANGLES, GU_VERTEX_32BITF|GU_TRANSFORM_3D, 12, 0, pyramid);
        sceGumPopMatrix();
    }

    /* item boxes */
    float spin = frameCount * 0.04f;
    for (int i = 0; i < NBOX; i++) {
        if (boxTimer[i] > 0) continue;
        ScePspFVector3 t = {boxPos[i].x, 2.2f + sinf(spin + i)*0.4f, boxPos[i].z};
        sceGumPushMatrix(); sceGumTranslate(&t); sceGumRotateY(spin); sceGumRotateX(spin*0.7f);
        box(0,0,0,2.2f,2.2f,2.2f,RGB(255,215,0));
        sceGumPopMatrix();
    }

    for (int k = 0; k < NKARTS; k++) drawKart(&karts[k]);
}

static void initGraphics(void) {
    void *fbp0 = (void*)0;
    void *fbp1 = (void*)(BUF_W*SCR_H*4);
    void *zbp  = (void*)(BUF_W*SCR_H*4*2);
    sceGuInit();
    sceGuStart(GU_DIRECT, dlist);
    sceGuDrawBuffer(GU_PSM_8888, fbp0, BUF_W);
    sceGuDispBuffer(SCR_W, SCR_H, fbp1, BUF_W);
    sceGuDepthBuffer(zbp, BUF_W);
    sceGuOffset(2048-(SCR_W/2), 2048-(SCR_H/2));
    sceGuViewport(2048, 2048, SCR_W, SCR_H);
    sceGuDepthRange(65535, 0);
    sceGuScissor(0, 0, SCR_W, SCR_H);
    sceGuEnable(GU_SCISSOR_TEST);
    sceGuDepthFunc(GU_GEQUAL);
    sceGuEnable(GU_DEPTH_TEST);
    sceGuShadeModel(GU_FLAT);
    sceGuDisable(GU_CULL_FACE);   /* skip winding headaches; fine at this poly count */
    sceGuFinish();
    sceGuSync(0, 0);
    sceDisplayWaitVblankStart();
    sceGuDisplay(GU_TRUE);
}

/* ---------- main ---------- */
int main(void) {
    setup_callbacks();
    pspDebugScreenInit();            /* used only for HUD text */
    initGraphics();
    sceCtrlSetSamplingCycle(0);
    sceCtrlSetSamplingMode(PSP_CTRL_MODE_ANALOG);

    buildMeshes();
    buildTrack();
    resetRace();

    void *drawOffset = (void*)0;     /* VRAM offset of the buffer being drawn */
    float camH = karts[0].h;

    while (1) {
        frameCount++;
        SceCtrlData pad;
        sceCtrlReadBufferPositive(&pad, 1);

        if (pad.Buttons & PSP_CTRL_START) { resetRace(); camH = karts[0].h; }

        /* player input */
        float steer = (pad.Lx - 128) / 128.f;
        if (fabsf(steer) < 0.15f) steer = 0.f;
        if (pad.Buttons & PSP_CTRL_LEFT)  steer = -1.f;
        if (pad.Buttons & PSP_CTRL_RIGHT) steer =  1.f;
        int accel = (pad.Buttons & PSP_CTRL_CROSS) != 0;
        int brake = (pad.Buttons & PSP_CTRL_SQUARE) != 0;
        int drift = (pad.Buttons & PSP_CTRL_RTRIGGER) != 0;
        int use   = (pad.Buttons & PSP_CTRL_TRIANGLE) != 0;

        if (countdown > 0) {
            countdown--;
            /* rolling start boost: hold X when "GO" appears */
            if (countdown == 60 && accel) karts[0].boost = 40;
        }
        if (countdown <= 60) {
            if (karts[0].finished) { accel = 0; steer = 0; }
            updateKart(&karts[0], steer, accel && !karts[0].finished, brake, drift, use);
            for (int k = 1; k < NKARTS; k++) aiControl(&karts[k], karts[0].total);
            collideKarts();
            updateItemBoxes();
        }

        /* chase camera */
        Kart *p = &karts[0];
        camH += wrapPi(p->h - camH) * 0.12f;
        float back = 10.f + p->speed * 2.f;
        ScePspFVector3 eye    = {p->x - sinf(camH)*back, 5.0f, p->z - cosf(camH)*back};
        ScePspFVector3 center = {p->x + sinf(camH)*6.f, 1.5f, p->z + cosf(camH)*6.f};
        ScePspFVector3 up     = {0,1,0};

        sceGuStart(GU_DIRECT, dlist);
        sceGuClearColor(RGB(120,180,240));
        sceGuClearDepth(0);
        sceGuClear(GU_COLOR_BUFFER_BIT | GU_DEPTH_BUFFER_BIT);

        float fov = 65.f + (p->boost > 0 ? 8.f : 0.f);
        sceGumMatrixMode(GU_PROJECTION);
        sceGumLoadIdentity();
        sceGumPerspective(fov, 16.f/9.f, 1.f, 450.f);
        sceGumMatrixMode(GU_VIEW);
        sceGumLoadIdentity();
        sceGumLookAt(&eye, &center, &up);
        sceGumMatrixMode(GU_MODEL);
        sceGumLoadIdentity();

        drawWorld(eye.x, eye.z);

        sceGuFinish();
        sceGuSync(0, 0);

        /* HUD */
        pspDebugScreenSetOffset((int)drawOffset);
        pspDebugScreenSetXY(1, 1);
        int lap = p->total < 0 ? 1 : p->total / NS + 1;
        if (lap > NLAPS) lap = NLAPS;
        pspDebugScreenSetTextColor(0xffffffff);
        pspDebugScreenPrintf("POS %d/%d   LAP %d/%d", placeOf(0), NKARTS, lap, NLAPS);
        pspDebugScreenSetXY(1, 3);
        pspDebugScreenPrintf("SPEED %3d km/h", (int)(fabsf(p->speed) * 150.f));
        pspDebugScreenSetXY(1, 4);
        pspDebugScreenPrintf("ITEM: %s", p->item ? "MUSHROOM (Triangle)" : "-");
        if (p->drift) {
            pspDebugScreenSetXY(1, 5);
            pspDebugScreenPrintf("DRIFT %s", p->charge > 90 ? "MAX!" : (p->charge > 45 ? "BLUE" : "..."));
        }
        if (countdown > 0) {
            int n = (countdown - 1) / 60;
            pspDebugScreenSetXY(28, 12);
            if (n > 0) pspDebugScreenPrintf("%d", n); else pspDebugScreenPrintf("GO!");
        }
        if (p->finished) {
            pspDebugScreenSetXY(18, 14);
            pspDebugScreenPrintf("FINISHED - PLACE %d", placeOf(0));
            pspDebugScreenSetXY(16, 16);
            pspDebugScreenPrintf("Press START to race again");
        }

        sceDisplayWaitVblankStart();
        drawOffset = sceGuSwapBuffers();
        /* sceGuSwapBuffers returns the new draw buffer's address */
        drawOffset = (void*)((unsigned int)drawOffset & 0x00ffffff);
    }
    return 0;
}
