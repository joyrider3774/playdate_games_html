/******************************************************************************
** XLander - A three-dimensional view-oriented lunar landing simulation for X
**
** Authors:
** Paul Riddle (paulr@umbc3.umbc.edu)
** Mike Friedman (mikef@umbc3.umbc.edu)
**
** University of Maryland, Baltimore Campus
** 
** This program may be freely distributed in any form, providing the authors'
** names stay with it.  If you use any portion of this code, please give us
** credit.  Let us know if you like it!
******************************************************************************/

/* ported to Playdate by gwem <cofinalsubnets@gmail.com> */

#include "pd_api.h"
#include <math.h>
#include <string.h>



#define PHI 1.618f // golden ratio -- used to scale difficulty
#define viewWidth 400            /* Width and height of view window */
#define viewHeight 180
#define panelHeight 60          /* Height of control panel */
#define WORLD_LENGTH 20000       /* Length of world in pixels */
#define WORLD_WIDTH 20000        /* Width of world in pixels */
#define HALF_WORLD_LENGTH (WORLD_LENGTH >> 1)
#define HALF_WORLD_WIDTH (WORLD_WIDTH >> 1)
#define PIXELS_PER_FOOT 6        /* Number of pixels per foot */
#define TICKS_PER_SECOND 4       /* Number of frames per second */
#define ACCELERATION -5.310      /* Acceleration of gravity (ft/sec^2) */
#define RETRO 35.0               /* Acceleration due to retroactive thruster */
#define LATERAL_THRUST 5.0       /* Acceleration due to lateral thruster */
#ifndef PI
#define PI 3.1415926535897932384f
#endif
#define HALFPI 1.5707963f         /* pi/2 */
#define TWOPI100 628
#define TWOPI 6.2831853f
#define VERT_SPEED 30.0f          /* Maximum vertical speed without crashing */
#define LAT_SPEED 30.0f           /* Maximum lateral speed without crashing */
#define RETRO_BURN 1.6f           /* Retroactive thruster fuel consumption */
#define LATERAL_BURN 0.4f         /* Lateral thruster fuel consumption */
#define FULL_TANK 320            /* Full tank of fuel */
#define MAX_VELOC 640.0f          /* Maximum velocity */

#define LANDER_WIDTH 600
#define LANDER_HEIGHT 600
#define PAD_WIDTH 750
#define PAD_HEIGHT 750

/*
 * Lines in 3-space
 */
typedef struct {
   float x1, y1, z1;
   float x2, y2, z2;
} LINE;

/*
 * Data structure for the "world".  The world consists of a linked list
 * of lines in 3-space, an array of segments used to plot the world in the
 * window, and the view orientation.
 */
struct dbentry {
   LINE line;
   struct dbentry *next;
};

// line in 2d display space
typedef struct {
  short x1, y1, x2, y2;
} LineSegment;

typedef struct database {
   struct dbentry *lines;
   int linecount, segcount;
   LineSegment *segments;
   int min_x, min_y, max_x, max_y;
   int off_x, off_y, off_z;
} DATABASE;

typedef struct {
   int px, py, pz;
   int pitch, roll, yaw;
   int front_thruster, rear_thruster;
   int left_thruster, right_thruster;
   int retro_thruster;
   char controls[7];
   float vert_speed, heading, lat_veloc;
   float fuel, alt;
} LANDER;

static int update(void* userdata);
const char* fontpath = "/System/Fonts/Roobert-10-Bold";
LCDFont* font = NULL;
PlaydateAPI *Pd;

static DATABASE world, craft, thrust, shadow;
static LANDER lander;

static void
  draw_label(int, int, int, const char*),
  nav_buttons(void),
  seed_rng(void),
  UpdateInstruments (float, float, float, int, int),
  Error(const char*),
  clear_buffer(void),
  *Malloc(size_t),
  Free(void*),
  DBInsert(DATABASE*, LINE*),
  DBPlot(DATABASE*),
  UpdateDisplay (void),
  SetupSinCosTable(void),
  InitializeLander(void),
  DisplayWorld(void),
  Pause(char*),
  DBFinish(DATABASE*),
  LoadWorld(void),
  DBInitFromData(DATABASE*, LINE*, int);

static bool paused;
static float
  retro_thrust = RETRO,
  acceleration = ACCELERATION,
  SIN[TWOPI100],
  COS[TWOPI100];
static int score = 0, text_width(const char*);
static LINE
  lander_data[] = {
     /* BODY */
     /* Top half */
     { 0, 800, 0, 300, 500, 0 },
     { 0, 800, 0, -300, 500, 0 },
     { 0, 800, 0, 0, 500, 300 },
     { 0, 800, 0, 0, 500, -300 },
     /* Sides */
     { 0, 500, 300, 300, 500, 0 },
     { 300, 500, 0, 0, 500, -300 },
     { 0, 500, -300, -300, 500, 0 },
     { -300, 500, 0, 0, 500, 300 },
     /* Bottom half */
     { 0, 200, 0, 300, 500, 0 },
     { 0, 200, 0, -300, 500, 0 },
     { 0, 200, 0, 0, 500, 300 },
     { 0, 200, 0, 0, 500, -300 },
     /* LEGS */
     { 300, 500, 0, 300, 0, 0 },
     { -300, 500, 0, -300, 0, 0 },
     { 0, 500, 300, 0, 0, 300 },
     { 0, 500, -300, 0, 0, -300 }, },
  // lander shadow
  shadow_data[] = {
     { 300, 0, 0, 0, 0, 300 },
     { 0, 0, 300, -300, 0, 0 },
     { -300, 0, 0, 0, 0, -300 },
     { 0, 0, -300, 300, 0, 0 }, },
  // thrust flames
  thrust_data[] = {
     { 0, 200, 0, 100, -100, 30 },
     { 0, 200, 0, -50, -100, 100 },
     { 0, 200, 0, 10, -100, 40 },
     { 0, 200, 0, -100, -100, -40 }, },
  landingpad[] = {
     { 0, 0, 0, 750, 0, 0 },
     { 0, 0, 0, 0, 0, 750, },
     { 750, 0, 0, 750, 0, 750 },
     { 0, 0, 750, 750, 0, 750 },
     { 0, 0, 0, 0, 750, 0 },
     { 0, 750, 0, 150, 750, 0 },
     { 150, 750, 0, 150, 550, 0 },
     { 150, 550, 0, 0, 550, 0 }, };
#define THRUSTSIZE (sizeof thrust_data / sizeof (LINE))
#define SHADOWSIZE (sizeof shadow_data / sizeof (LINE))
#define PADSIZE (sizeof landingpad / sizeof (LINE))
#define LANDERSIZE (sizeof lander_data / sizeof (LINE))

int eventHandler(PlaydateAPI* pd, PDSystemEvent event, uint32_t _arg) {
	if ( event == kEventInit ) {
    Pd = pd;
    pd->display->setInverted(1);
    font = pd->graphics->loadFont(fontpath, NULL);
		pd->graphics->setFont(font);
    pd->graphics->setDrawMode(kDrawModeFillWhite);
    LoadWorld();
    DBInitFromData(&thrust, thrust_data, THRUSTSIZE);
    DBInitFromData(&shadow, shadow_data, SHADOWSIZE);
    DBInitFromData(&craft, lander_data, LANDERSIZE);
    InitializeLander();
    SetupSinCosTable ();
    Pause("Press any button to begin");
		pd->system->setUpdateCallback(update, pd);
	}
	return 0;
}

static int update(void* userdata) {
  if (paused) {
    PDButtons current, pushed, released;
    Pd->system->getButtonState(&current, &pushed, &released);
    if (pushed) paused = false, InitializeLander();
  } else {
    Pd->graphics->clear(kColorWhite);
    Pd->graphics->setScreenClipRect(0, 0, 400, 240);
    UpdateDisplay(); }
  Pd->graphics->markUpdatedRows(0, LCD_ROWS);
	return 1; }

/*
 * Transform x,y coordinate pairs from world space to position on
 * "radar" screen
 */
#define WorldToRadarX(x) (180 + (((int) (x) + (WORLD_WIDTH >> 1)) / 384))
#define WorldToRadarY(y) (viewHeight + 56 - (((int) (y) + (WORLD_LENGTH >> 1)) / 384))

static void CrunchLander(void) {
   char *s;
   Pd->system->formatString(&s, "CRASH!!     Final Score: %d", score);
   Pause(s);
   Free(s);
   score = 0;
   acceleration = ACCELERATION;
   retro_thrust = RETRO; }

/******************************************************************************
** RateLanding
**
** Award points based on how well the player landed
******************************************************************************/

static void RateLanding(void) {
   int x_distance = abs((int) (craft.off_x - (landingpad[0].x1 + (PAD_WIDTH >> 1)))),
       z_distance = abs((int) (craft.off_z - (landingpad[0].z1 + (PAD_HEIGHT >> 1))));
   char *s;
   if (x_distance > 250 || z_distance > 250)
     Pd->system->formatString(&s, "Landed Off Pad!  Score: %d", score);
   else {
     score += (int) (100.0f * (1 - ((lander.vert_speed + lander.lat_veloc) / 80.0f))),
     Pd->system->formatString(&s, "Nice Landing!  Score: %d", score);
     /*
      * Make things a little tougher
      */
     acceleration *= PHI;
     retro_thrust *= PHI; }
   Pause(s);
   Free(s); }



static void LoadPerspectiveMatrix (float CTM[4][4], float z) {
   float Tx, Ty, fd = 200.0, S = z + fd ? fd / (z + fd) : 0;
   Tx = (1.0f - S) * (viewWidth >> 1);
   Ty = (1.0f - S) * (viewWidth >> 2);
   CTM[0][0] = S;
   CTM[3][0] = Tx;
   CTM[1][1] = S;
   CTM[3][1] = Ty; }

static void MultPerspective (float CM[], float CTM[4][4],float RM[]) {
   RM[0]=CTM[0][0]*CM[0]+CTM[1][0]*CM[1]+CTM[2][0]*CM[2]+CTM[3][0]*CM[3];
   RM[1]=CTM[0][1]*CM[0]+CTM[1][1]*CM[1]+CTM[2][1]*CM[2]+CTM[3][1]*CM[3];
   RM[2]=CTM[0][2]*CM[0]+CTM[1][2]*CM[1]+CTM[2][2]*CM[2]+CTM[3][2]*CM[3];
   RM[3]=CTM[0][3]*CM[0]+CTM[1][3]*CM[1]+CTM[2][3]*CM[2]+CTM[3][3]*CM[3]; }

static int Clip (float CM1[], float CM2[], float RM1[], float RM2[]) {
  const int crash = 2;
  if ((CM1[2]< 0) && (CM2[2]< 0)) return 0;   /***  line is completely behind plane        ***/
  if ((CM1[2]>= 0) && (CM2[2]>= 0)) return 1; /***  line is in front of plane,no clipping  ***/
  if (CM1[2] == CM2[2]) return 1;
  float *rm = CM1[2] < 0 ? RM1 : RM2;
   
  rm[0] = CM2[0] - ((CM1[0]-CM2[0])/(CM1[2]-CM2[2]))*CM2[2];
  rm[1] = CM2[1] - ((CM1[1]-CM2[1])/(CM1[2]-CM2[2]))*CM2[2];
  rm[2] = 0; 
  /***  calculate intersection point (x,y,0)   ***/
  if (((rm[0] < viewWidth) && (rm[0] > 0)) && ((rm[1] < viewHeight) && (rm[1] > 0)))
    return crash;
  return 1; }

static int WorldToDisplay(LINE*line, LineSegment*segment) {
   float  VP1[4],VP2[4];             /* viewing coordinates                  */
   float  NP1[4],NP2[4];             /* normalized coordinates               */
   register float  tx1,tx2;          /* temporary storage for view transform */
   register float  ty1,ty2;
   register float  tz1,tz2;
   static float PTM[4][4] = {
      { 0.0, 0.0, 0.0, 0.0 },     /* matrix for perspective and norm. coord. */
      { 0.0, 0.0, 0.0, 0.0 },
      { 0.0, 0.0, 1.0, 0.0 },
      { 0.0, 0.0, 0.0, 1.0 },
   };
   int px = lander.px;
   int py = lander.py;
   int pz = lander.pz;
   int pitch = lander.pitch;
   int roll = lander.roll;
   int yaw = lander.yaw;

   /*
    * Wrap the stuff that the user can't see.  This gives the illusion
    * of an "infinite" world.  (Well, almost...)
    */
   if (line->z1 < lander.pz)
      line->z1 += WORLD_LENGTH,
      line->z2 += WORLD_LENGTH;
   else if (line->z1 > lander.pz + WORLD_LENGTH)
      line->z1 -= WORLD_LENGTH,
      line->z2 -= WORLD_LENGTH;

   if (line->x1 < lander.px - (WORLD_WIDTH >> 1))
      line->x1 += WORLD_WIDTH,
      line->x2 += WORLD_WIDTH;
   else if (line->x1 > lander.px + (WORLD_WIDTH >> 1))
      line->x1 -= WORLD_WIDTH,
      line->x2 -= WORLD_WIDTH;

   /*
    * Translate viewpoint to world origin
    */
   VP1[0] = line->x1 - px;
   VP1[1] = line->y1 - py;
   VP1[2] = line->z1 - pz;
   VP1[3] = 1;
   VP2[0] = line->x2 - px;
   VP2[1] = line->y2 - py;
   VP2[2] = line->z2 - pz;
   VP2[3] = 1;

   /*
    * orient the view axes with world axes
    */
   tx1 = VP1[0]*(COS[roll]*COS[yaw] + SIN[roll]*SIN[pitch]*SIN[yaw]) +
         VP1[1]*(-SIN[roll]*COS[yaw] + COS[roll]*SIN[pitch]*SIN[yaw]) +
	       VP1[2]*COS[pitch]*SIN[yaw];
      
   tx2 = VP2[0]*(COS[roll]*COS[yaw] + SIN[roll]*SIN[pitch]*SIN[yaw]) +
         VP2[1]*(-SIN[roll]*COS[yaw] + COS[roll]*SIN[pitch]*SIN[yaw]) +
         VP2[2]*COS[pitch]*SIN[yaw];
      
   ty1 = VP1[0]*SIN[roll]*COS[pitch] +
         VP1[1]*COS[roll]*COS[pitch] -
         VP1[2]*SIN[pitch];
   
   ty2 = VP2[0]*SIN[roll]*COS[pitch] +
         VP2[1]*COS[roll]*COS[pitch] -
         VP2[2]*SIN[pitch];
   
   tz1 = VP1[0]*(-COS[roll]*SIN[yaw] + SIN[roll]*SIN[pitch]*COS[yaw]) +
         VP1[1]*(SIN[roll]*SIN[yaw] + COS[roll]*SIN[pitch]*COS[yaw]) +
	       VP1[2]*COS[pitch]*COS[yaw];
   
   tz2 = VP2[0]*(-COS[roll]*SIN[yaw] + SIN[roll]*SIN[pitch]*COS[yaw]) +
         VP2[1]*(SIN[roll]*SIN[yaw] + COS[roll]*SIN[pitch]*COS[yaw]) +
	       VP2[2]*COS[pitch]*COS[yaw];


   VP1[0] = tx1 + (viewWidth >> 1);
   VP2[0] = tx2 + (viewWidth >> 1);
   VP1[1] = ty1;
   VP2[1] = ty2;
   VP1[2] = tz1;
   VP2[2] = tz2;
   
   /*
    * Clip the line segment at z = 0
    */
   if (Clip(VP1,VP2,VP1,VP2) == 0) return 0;

   /*
    * Do perspective projection
    */
   LoadPerspectiveMatrix (PTM,VP1[2]);
   MultPerspective (VP1,PTM,NP1);
   LoadPerspectiveMatrix (PTM,VP2[2]);
   MultPerspective (VP2,PTM,NP2);

   NP1[1] = (float) viewHeight - NP1[1];
   NP2[1] = (float) viewHeight - NP2[1];

   /*
    * clip all lines completely out of window
    */
   if ((NP1[0] < 0.0f && NP2[0] < 0.0f) ||
       (NP1[1] < 0.0f && NP2[1] < 0.0f) ||
       (NP1[0] > (float) viewWidth && NP2[0] > (float) viewWidth) ||
       (NP1[1] > (float) viewHeight && NP2[1] > (float) viewHeight))
      return 0;

   /*
    * convert to device coordinates
    */
   segment->x1 = (short) NP1[0];
   segment->y1 = (short) NP1[1];
   segment->x2 = (short) NP2[0];
   segment->y2 = (short) NP2[1];
   return 1;
}

/******************************************************************************
** StoreLine
**
** Inserts the given line segment into the XSegment array for later plotting.
******************************************************************************/

static void StoreLine(DATABASE*database, LineSegment *segment) {
   memmove(&database->segments[database->segcount++], segment, sizeof (LineSegment));
}


/******************************************************************************
** DBPlot
**
** This plots the entire line database in the background buffer.  Then zeros
** out the segment index so we can store more lines for the next view.
******************************************************************************/

static void DBPlot(DATABASE *db) {
   struct dbentry *entry;
   LineSegment segment;
   LINE line;

   for (entry = db->lines; entry; entry = entry->next) {
      line.x1 = entry->line.x1 + db->off_x;
      line.x2 = entry->line.x2 + db->off_x;
      line.y1 = entry->line.y1 + db->off_y;
      line.y2 = entry->line.y2 + db->off_y;
      line.z1 = entry->line.z1 + db->off_z;
      line.z2 = entry->line.z2 + db->off_z;
      if (WorldToDisplay(&line, &segment)) StoreLine(db, &segment);
   }
   if (!(db->segcount)) return;
   for (int i = db->segcount; i--;) {
     LineSegment seg = db->segments[i];
     Pd->graphics->drawLine(seg.x1, seg.y1, seg.x2, seg.y2, 1, kColorBlack); }
   db->segcount = 0;
}




/******************************************************************************
** DBInitFromData
**
** Create a pre-loaded database from a list of line segments
******************************************************************************/

static void DBInitFromData(DATABASE *db, LINE*lines, int nlines) {
   for (int count = 0; count < nlines; count++)
      DBInsert (db, &lines[count]);
   DBFinish(db); }

/******************************************************************************
** DBFinish
**
** This routine should be called when all entries have been inserted into
** the database.  It allocates space for the 2-d segment array used internally
** by the plotting routines.
******************************************************************************/

static void DBFinish(DATABASE*database) {
   if (!(database->segments = (LineSegment*) Malloc(database->linecount * sizeof (LineSegment))))
     Error("Insufficient memory"); }

/******************************************************************************
** DBFree
**
** This frees up the entire database
******************************************************************************/

void DBFree(DATABASE*database) {
   struct dbentry *entry = database->lines, *temp;
   while (entry)
      temp = entry,
      entry = entry->next,
      Free(temp);
   Free(database->segments); }

/******************************************************************************
** DBInsert
**
** DBInsert takes a pointer to a LINE, creates a new database entry, and
** adds it to the supplied database.
******************************************************************************/

static void DBInsert(DATABASE*database, LINE*line) {
   struct dbentry *entry = Malloc(sizeof (struct dbentry));
   if (!entry) Error("Insufficient memory");
   database->linecount++;
   memcpy(&entry->line, line, sizeof (LINE));
   entry->next = database->lines;
   database->lines = entry; }


#define EDGE_LENGTH 1000

/******************************************************************************
** InitializeLander
**
** Set up initial values for the lander when beginning a new game or
** a new landing.
******************************************************************************/

static void InitializeLander(void) {
   lander.px = craft.off_x = 0;
   lander.py = craft.off_y = 8000;
   craft.off_z = -HALF_WORLD_LENGTH;
   lander.pz = craft.off_z - 2000;
   lander.front_thruster = lander.rear_thruster = 0;
   lander.left_thruster = lander.right_thruster = 0;
   lander.retro_thruster = 0;
   lander.vert_speed = 0.0;
   lander.heading = 1.36;
   lander.lat_veloc = 100.0;
   lander.fuel = 320.0;
   lander.alt = craft.off_y / PIXELS_PER_FOOT; }

/******************************************************************************
** SetupSinCosTable
**
** This loads a table with sine and cosine values.  The table is later
** accessed to speed up computations when plotting.
******************************************************************************/

static void SetupSinCosTable(void) {
   int i;
   float t = 0.0;
   for (i = 0; i < TWOPI100; i++, t += TWOPI / TWOPI100)
      SIN[i] = sinf(t),
      COS[i] = cosf(t); }

static void DisplayWorld (void) {
   Pd->graphics->setScreenClipRect(0, 0, 400, 180);
   DBPlot(&world);
   DBPlot(&craft);

   /*
    * Display thrust flames if we are thrusting
    */
   if (lander.retro_thruster != 0) {
      thrust.off_x = craft.off_x;
      thrust.off_y = craft.off_y;
      thrust.off_z = craft.off_z;
      DBPlot(&thrust);
   }
   shadow.off_x = craft.off_x;
   shadow.off_z = craft.off_z;
   DBPlot(&shadow);
}

static void LoadWorld(void) {
   int count = 0, x, y, r = 0, height, goal_x, goal_y;
   float x_offset = (float) EDGE_LENGTH * cosf(60.0f * PI / 180.0f);
   float y_offset = (float) EDGE_LENGTH * sinf(60.0f * PI / 180.0f);
   LINE line;

   world.min_x = world.min_y = -HALF_WORLD_LENGTH;
   world.max_x = world.max_y = HALF_WORLD_WIDTH;
   seed_rng();
   for (x = -HALF_WORLD_WIDTH; x < HALF_WORLD_WIDTH; r ^= 1, x += EDGE_LENGTH + (int) x_offset)
      for (y = r * (int) y_offset - HALF_WORLD_LENGTH; y < HALF_WORLD_LENGTH; y += (int) (2.0f * y_offset))
        height = rand() % 3 ? 0 : rand() % 2500,
        line.x1 = x,
        line.y1 = height,
        line.z1 = y,
        line.x2 = x + EDGE_LENGTH,
        line.y2 = 0,
        line.z2 = y,
        DBInsert(&world, &line),
        line.x1 = x,
        line.y1 = height,
        line.z1 = y,
        line.x2 = x - (int) x_offset,
        line.y2 = 0,
        line.z2 = y - (int) y_offset,
        DBInsert(&world, &line),
        line.x1 = x,
        line.y1 = height,
        line.z1 = y,
        line.x2 = x - (int) x_offset,
        line.y2 = 0,
        line.z2 = y + (int) y_offset,
        DBInsert(&world, &line);

   /*
    * Drop the landing pad "somewhere on the ground"
    */
   goal_x = (rand () % WORLD_WIDTH) - HALF_WORLD_WIDTH;
   goal_y = (rand () % WORLD_LENGTH) - HALF_WORLD_LENGTH;
   for (count = 0; count < PADSIZE; count++)
      landingpad[count].x1 += goal_x,
      landingpad[count].x2 += goal_x,
      landingpad[count].z1 += goal_y,
      landingpad[count].z2 += goal_y,
      DBInsert(&world, &landingpad[count]);
   DBFinish(&world); }


static LCDPattern panel_bg = {
    // Bitmap
  0b10101010,
  0b01010101,
  0b10101010,
  0b01010101,
  0b10101010,
  0b01010101,
  0b10101010,
  0b01010101,
  // Mask
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
};
static void UpdateDisplay(void) {
  Pd->graphics->fillRect(0, viewHeight, viewWidth, panelHeight, (LCDColor) panel_bg);

  /* Heading indicator */
  Pd->graphics->fillEllipse(4, 184, 52, 52, 0.0f, 359.0f, kColorWhite);

  /* "Radar" display */
  Pd->graphics->fillRect(180, 184, 52, 52, kColorWhite);
   draw_label(60, viewHeight + 22, 2, "Heading");
   char *s;
   Pd->system->formatString(&s, "Gravity: %.2f ft*s^-2", (double) acceleration);
   draw_label(240, 201, 2, s);
   Free(s);
   Pd->system->formatString(&s, "Score: %d", score);
   draw_label(240, 183, 2, s);
   Free(s);
   Pd->system->formatString(&s, "Velocity: %06.2f ft/s", (double) lander.vert_speed);
   draw_label(240, 219, 2, s);
   Free(s);
   float lat_accel_x, lat_accel_y;
   float lat_veloc_x, lat_veloc_y;
   nav_buttons();

   if (lander.retro_thruster > 0) lander.fuel -= RETRO_BURN;
   if (lander.front_thruster > 0) lander.fuel -= LATERAL_BURN;
   if (lander.rear_thruster > 0)  lander.fuel -= LATERAL_BURN;
   if (lander.left_thruster > 0)  lander.fuel -= LATERAL_BURN;
   if (lander.right_thruster > 0) lander.fuel -= LATERAL_BURN;

   lander.vert_speed += (lander.retro_thruster + acceleration) / TICKS_PER_SECOND;
   lander.alt += lander.vert_speed / TICKS_PER_SECOND;
   lat_accel_x = lander.right_thruster - lander.left_thruster;
   lat_accel_y = lander.rear_thruster - lander.front_thruster;
   lat_veloc_x = lander.lat_veloc * cosf(lander.heading) + lat_accel_x;
   lat_veloc_y = lander.lat_veloc * sinf(lander.heading) + lat_accel_y;
   craft.off_x += (lat_veloc_x / TICKS_PER_SECOND) * PIXELS_PER_FOOT;
   craft.off_z += (lat_veloc_y / TICKS_PER_SECOND) * PIXELS_PER_FOOT;
   lander.lat_veloc = sqrt(lat_veloc_x * lat_veloc_x + lat_veloc_y * lat_veloc_y);

   if (lander.lat_veloc > MAX_VELOC) lander.lat_veloc = MAX_VELOC;
   if (lander.vert_speed > MAX_VELOC) lander.vert_speed = MAX_VELOC;

   lander.heading = (lat_veloc_x != 0.0f) ? atan2f(lat_veloc_y, lat_veloc_x) : 0.0f;

   /*
    * Wrap the coordinates around as the craft hits the boundaries
    * of the "world".  Also update the view so that it follows the craft
    * around with a little bit of lag.  This gives a pretty neat effect.
    */

   lander.px += (craft.off_x - lander.px) / 5;
   lander.py += (craft.off_y - lander.py) / 5;
   lander.pz += (craft.off_z - 2000 - lander.pz) / 5;

   if (craft.off_x < world.min_x)
      craft.off_x = world.max_x,
      lander.px += WORLD_WIDTH;
   else if (craft.off_x > world.max_x)
      craft.off_x = world.min_x,
      lander.px -= WORLD_WIDTH;

   if (craft.off_z < world.min_y)
      craft.off_z = world.max_y,
      lander.pz += WORLD_LENGTH;
   else if (craft.off_z > world.max_y)
      craft.off_z = world.min_y,
      lander.pz -= WORLD_LENGTH;
   
   if (lander.heading < 0.0f) lander.heading += TWOPI;
   else if (lander.heading > TWOPI) lander.heading -= TWOPI;

   craft.off_y = (int) lander.alt * PIXELS_PER_FOOT;
   UpdateInstruments(lander.heading, lander.vert_speed, lander.fuel, craft.off_x, craft.off_z);
   DisplayWorld();
   if (lander.alt < 0.0f) {
      if (-lander.vert_speed > VERT_SPEED || lander.lat_veloc > LAT_SPEED)
        CrunchLander();
      else RateLanding(); } }

static void UpdateInstruments (float heading, float roc, float fuel, int x, int y) {
   static int heading_x = 50, heading_y = 15;
   static int fuel_level = 53;
   int new_fuel_level = (int) fuel / 6;
   char buf[32];

   /*
    * Update heading indicator
    */
   heading_x = 30 + 24 * cosf(heading);
   heading_y = 30 - 24 * sinf(heading);
   Pd->graphics->drawLine(30, 210, heading_x, viewHeight + heading_y, 2, kColorXOR);

   /*
    * Update fuel gauge
    */
   Pd->graphics->fillRect(140, viewHeight + 4, 20, 53, kColorWhite);
   Pd->graphics->fillRect(140, viewHeight + 4 + 53 - fuel_level, 20, fuel_level, kColorXOR);
   fuel_level = new_fuel_level;
   draw_label(160, 184, 2, "F");
   draw_label(160, 219, 2, "E");

   /*
    * Display vertical speed
    */

   /*
    * Update "radar"
    */
   Pd->graphics->setScreenClipRect(180, 184, 52, 52);
   Pd->graphics->fillEllipse(
     WorldToRadarX(landingpad[0].x1),
     WorldToRadarY(landingpad[0].z1), 5, 5, 0.0f, 360.0f, kColorXOR);
   Pd->graphics->fillEllipse(
       WorldToRadarX(x),
       WorldToRadarY(y), 3, 3, 0.0f, 360.0f, kColorXOR); }

static int text_width(const char*s) {
  return Pd->graphics->getTextWidth(font, s, strlen(s), kASCIIEncoding, 0); }
static int text_height(const char*s) {
  return Pd->graphics->getTextHeightForMaxWidth(font, s, strlen(s), 1000, kASCIIEncoding, kWrapClip, 0, 0); }

static void draw_label(int x, int y, int pad, const char *text) {
  int width = text_width(text) + 2 * pad, height = text_height(text) + 2 * pad;
  Pd->graphics->fillRect(x, y, width, height, kColorBlack);
  Pd->graphics->drawText(text, strlen(text), kASCIIEncoding, x + pad, y + pad); }

static void seed_rng(void) { srand((long)Pd->system->getCurrentTimeMilliseconds()); }
static void *Malloc(size_t n) { return Pd->system->realloc(NULL, n); }
static void Free(void*x) { Pd->system->realloc(x, 0); }
static void Error(const char *s) { Pd->system->error("Error:  %s\n", s); }

static void Pause(char *msg) {
  paused = true;
  int y = 116 - text_height(msg) / 2,
      x = 196 - text_width(msg) / 2;
  draw_label(x, y, 2, msg); }

static void nav_buttons(void) {
  PDButtons current, pushed, released;
  Pd->system->getButtonState(&current, &pushed, &released);
  float angle = Pd->system->getCrankChange();
  if (lander.fuel <= 0.0f)
    lander.rear_thruster = lander.front_thruster = lander.left_thruster = lander.right_thruster = lander.retro_thruster = 0;
  else {
    lander.rear_thruster = current & kButtonUp ? LATERAL_THRUST : 0;
    lander.front_thruster = current & kButtonDown ? LATERAL_THRUST : 0;
    lander.left_thruster = current & kButtonLeft ? LATERAL_THRUST : 0;
    lander.right_thruster = current & kButtonRight ? LATERAL_THRUST : 0;
    lander.retro_thruster = current & (kButtonA | kButtonB) || angle > 0 ?  retro_thrust : 0; } }
