#include "StepperLog.h"
#include "StepperCore.h"
#include "StepperDriver.h"
#include "StepperGenerator.h"


//Stepper::Core core;
//Stepper::Task task;
Stepper::Driver driver(26,25,27,false);
Stepper::Generator generator(driver);

// --- Simple Serial interface state ---
static long g_velocity = 3000;   // steps per second (example)
static int  g_steps    = 200;    // number of steps
static int  g_dir      = 1;      // 1=cw, -1=ccw

static char   g_lineBuf[128];
static size_t g_lineLen = 0;

static void printHelp() {
  Serial.println("Commands:");
  Serial.println("  v=<int>         set velocity (e.g., v=1200)");
  Serial.println("  s=<int>         set steps    (e.g., s=800)");
  Serial.println("  d=<cw|ccw|1|-1> set direction");
  Serial.println("  move <v> <s> <cw|ccw|1|-1>   set all and run");
  Serial.println("  go               run with current settings");
  Serial.println("  status           show current settings");
}

static void runMove() {
  Serial.print("Running: vel=");
  Serial.print(g_velocity);
  Serial.print(" steps=");
  Serial.print(g_steps);
  Serial.print(" dir=");
  Serial.println(g_dir > 0 ? "cw" : "ccw");



  // generator.moveSteps((uint64_t)g_steps,
  //                     (float)g_velocity,
  //                     10.0f,
  //                     10.0f,
  //                     g_dir > 0 ? Stepper::Direction::CLOCKWISE : Stepper::Direction::COUNTERCLOCKWISE);
  generator.run((float)g_velocity,
                      1000.0f,
                      1000.0f,
                      g_dir > 0 ? Stepper::Direction::CLOCKWISE : Stepper::Direction::COUNTERCLOCKWISE);
}

static bool parseDirToken(const char* v, int* outDir) {
  if (!v || !outDir) return false;
  if (strcmp(v, "cw") == 0 || strcmp(v, "1") == 0 || strcmp(v, "+") == 0) { *outDir = 1; return true; }
  if (strcmp(v, "ccw") == 0 || strcmp(v, "-1") == 0 || strcmp(v, "-") == 0) { *outDir = -1; return true; }
  return false;
}

static void processLine(char* line) {
  // Normalize spaces
  for (char* p = line; *p; ++p) if (*p == '\t') *p = ' ';

  // Tokenize by spaces
  bool gotV = false, gotS = false, gotD = false, doRun = false;
  char* saveptr = nullptr;
  for (char* tok = strtok_r(line, " ", &saveptr); tok; tok = strtok_r(nullptr, " ", &saveptr)) {
    // key=value form
    char* eq = strchr(tok, '=');
    if (eq) {
      *eq = '\0';
      const char* key = tok;
      const char* val = eq + 1;
      // lowercase key
      for (char* p = (char*)key; *p; ++p) *p = tolower(*p);

      if (strcmp(key, "v") == 0 || strcmp(key, "vel") == 0 || strcmp(key, "velocity") == 0) {
        g_velocity = strtol(val, nullptr, 10);
        gotV = true;
      } else if (strcmp(key, "s") == 0 || strcmp(key, "steps") == 0) {
        g_steps = (int)strtol(val, nullptr, 10);
        gotS = true;
      } else if (strcmp(key, "d") == 0 || strcmp(key, "dir") == 0 || strcmp(key, "direction") == 0) {
        int dirTmp;
        if (parseDirToken(val, &dirTmp)) { g_dir = dirTmp; gotD = true; }
      }
      continue;
    }

    // single-word commands
    for (char* p = tok; *p; ++p) *p = tolower(*p);
    if (strcmp(tok, "go") == 0 || strcmp(tok, "run") == 0) {
      doRun = true;
      continue;
    }
    if (strcmp(tok, "status") == 0) {
      Serial.print("vel="); Serial.print(g_velocity);
      Serial.print(" steps="); Serial.print(g_steps);
      Serial.print(" dir="); Serial.println(g_dir > 0 ? "cw" : "ccw");
      continue;
    }
    if (strcmp(tok, "help") == 0) {
      printHelp();
      continue;
    }
    if (strcmp(tok, "move") == 0) {
      // expect: move <v> <s> <dir>
      char* vTok = strtok_r(nullptr, " ", &saveptr);
      char* sTok = strtok_r(nullptr, " ", &saveptr);
      char* dTok = strtok_r(nullptr, " ", &saveptr);
      if (vTok && sTok && dTok) {
        g_velocity = strtol(vTok, nullptr, 10);
        g_steps    = (int)strtol(sTok, nullptr, 10);
        int dirTmp;
        if (parseDirToken(dTok, &dirTmp)) { 
          g_dir = dirTmp;
          doRun = true;
        }
      }
      continue;
    }
  }

  if (gotV || gotS || gotD) {
    Serial.print("Set: vel="); Serial.print(g_velocity);
    Serial.print(" steps="); Serial.print(g_steps);
    Serial.print(" dir="); Serial.println(g_dir > 0 ? "cw" : "ccw");
  }
  if (doRun || (gotV && gotS && gotD)) runMove();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  esp_log_level_set("*", ESP_LOG_INFO);

  while (!Serial) { vTaskDelay(pdMS_TO_TICKS(100)); }
  printHelp();
  //core.start();

  driver.setTimings(2, 2, 1, 100000);
  driver.start();
  vTaskDelay(pdMS_TO_TICKS(100));
  driver.enable();
  vTaskDelay(pdMS_TO_TICKS(100));
  driver.setDirection(Stepper::Direction::COUNTERCLOCKWISE);
  generator.start();
  // vTaskDelay(pdMS_TO_TICKS(1000));
  // generator.setVelocity(1.0f, 1000.0f, 1000.0f, Stepper::Direction::CLOCKWISE);
}

void loop() {

  //esp_err_t err = Stepper::taskEventLoopPost(Stepper::TaskEventId::NEW_TASK_EVENT, &task, sizeof(struct Stepper::Task), 1000);
  //ESP_LOGE("Main", "Posting event %s", esp_err_to_name(err));
  
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_lineLen > 0) {
        g_lineBuf[g_lineLen] = '\0';
        processLine(g_lineBuf);
        g_lineLen = 0;
      }
    } else if (g_lineLen < sizeof(g_lineBuf) - 1) {
      g_lineBuf[g_lineLen++] = c;
    }
  }

  vTaskDelay(pdMS_TO_TICKS(100));
}
