#include <PS2X_lib.h>  
#include <Servo.h>           
#include "bsp_motor_iic.hpp" // 引入 I2C 馬達模組 (完整保留)

// 🌟 1. 正式引入馬達控制庫通訊標頭檔
#include <Usb.h>
#include <cdcacm.h>
#include <damiao_control.hpp> 

// ============ PS2 手把引腳定義 ============
#define PS2_DAT         4
#define PS2_CMD         5
#define PS2_SEL         6
#define PS2_CLK         7
#define pressures   false
#define rumble      false

// ============ 類比與數位腳位定義 ============
#define LASER_PIN_A2    A2  // A2 負責純高低電位（雷射開關）
#define SERVO_3_PIN     A3  

// 定義自動化所需的偵測腳位
#define SENSOR_PIN_3    3    
#define SENSOR_PIN_2    2    

// ============ Yahboom 520 馬達參數 ============
#define MOTOR_TYPE_520  1   
#define GEAR_RATIO      30  
#define PULSE_LINE      11  
#define WHEEL_DIA       68.00 

#define MAX_PWM_LIMIT   1000  
#define MICRO_TURN_PWM  450  

// 所有的自動化行駛與轉彎，速度全部統一為 600
#define AUTO_DRIVE_PWM  600
#define AUTO_TURN_PWM   600

// ============ 🌟 大廟官方庫硬體與馬達物件實例化 ============
USB Usb;
class ACMAsyncOper : public CDCAsyncOper {
public:
    uint8_t OnInit(ACM *pacm) override { return 0; }
};
ACMAsyncOper AsyncOper;
ACM Acm(&Usb, &AsyncOper);

// 建立全域唯一的單一萬能控制器管理通訊
damiao::Motor_Control ctrl(&Usb, &Acm);

// 建立兩個速度模式下的馬達物件實例 (ID: 0x03, ID: 0x04)
damiao::Motor_Vel motor3_vel(0x03, 435.0f);//395.0f
damiao::Motor_Vel motor4_vel(0x04, 435.0f);

PS2X ps2x;
Servo myServo3;             
int error = 0;

uint16_t current_deadzone = 50;

// ============ 自動化狀態機變數 ============
bool is_auto_mode = false;     // 是否處於自動模式
int auto_state = 0;            // 狀態變數
unsigned long state_timer = 0; // 用於非阻塞型時間等待
bool case2_part_b = false;     // 標記 Case 2 的 Part B 左轉是否啟動
bool case5_part_b = false;     // 標記 Case 5 的 Part B（轉後直行）是否啟動

bool laser_state_a2 = false;   // A2 雷射狀態：false=LOW, true=HIGH (開機預設關)

// ============ 初始化 PS2 手把 ============
void initPS2(){
  delay(300);
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  if(error == 0){
    Serial.println("Found Controller, configured successful");
  }  
  else {
    Serial.println("No controller found, check wiring.");
  }
}

// ============ 核心功能：純 PWM 開環驅動底盤控制 ============
void control_chassis_joysticks() {
  uint8_t raw_ly = ps2x.Analog(PSS_LY);
  uint8_t raw_lx = ps2x.Analog(PSS_LX);
  uint8_t raw_ry = ps2x.Analog(PSS_RY);
  uint8_t raw_rx = ps2x.Analog(PSS_RX);
  
  int16_t left_forward = 0;
  int16_t left_turn = 0;
  int16_t right_forward = 0;
  int16_t right_turn = 0;

  if (raw_ly < 116)       left_forward = map(raw_ly, 116, 0, 0, MAX_PWM_LIMIT);
  else if (raw_ly > 140)  left_forward = map(raw_ly, 140, 255, 0, -MAX_PWM_LIMIT);
  if (raw_lx < 116)       left_turn = map(raw_lx, 116, 0, 0, -MAX_PWM_LIMIT);
  else if (raw_lx > 140)  left_turn = map(raw_lx, 140, 255, 0, MAX_PWM_LIMIT);

  if (raw_ry < 116)       right_forward = map(raw_ry, 116, 0, 0, MAX_PWM_LIMIT);
  else if (raw_ry > 140)  right_forward = map(raw_ry, 140, 255, 0, -MAX_PWM_LIMIT);
  if (raw_rx < 116)       right_turn = map(raw_rx, 116, 0, 0, -MAX_PWM_LIMIT);
  else if (raw_rx > 140)  right_turn = map(raw_rx, 140, 255, 0, MAX_PWM_LIMIT);

  int16_t total_forward = left_forward + right_forward;
  int16_t total_turn    = left_turn + right_turn;

  int16_t target_pwm_m1 = total_forward + total_turn;
  int16_t target_pwm_m2 = -(total_forward - total_turn); 
  
  target_pwm_m1 = constrain(target_pwm_m1, -MAX_PWM_LIMIT, MAX_PWM_LIMIT);
  target_pwm_m2 = constrain(target_pwm_m2, -MAX_PWM_LIMIT, MAX_PWM_LIMIT);
  
  control_pwm(target_pwm_m1, target_pwm_m2, 0, 0);
}

// ============ 自動化程式狀態機 ============
void run_auto_program() {
  switch (auto_state) {
    
    case 1: 
      Serial.println("Auto State 1: Moving Forward...");
      control_pwm(AUTO_DRIVE_PWM, -AUTO_DRIVE_PWM, 0, 0); 
      
      if (digitalRead(SENSOR_PIN_3) == HIGH) {
        Serial.println("D3 High Detected! Stopping for 0.5s...");
        control_pwm(0, 0, 0, 0); 
        state_timer = millis();  
        case2_part_b = false;    
        auto_state = 2;          
      }
      break;

    case 2: 
      if (!case2_part_b) {
        if (millis() - state_timer < 900) {
          control_pwm(AUTO_TURN_PWM, -AUTO_TURN_PWM, 0, 0); 
          break; 
        }
        
        Serial.println("Auto State 2 (Part A): Turning Right...");
        control_pwm(AUTO_TURN_PWM, AUTO_TURN_PWM, 0, 0); 
        
        if (millis() - state_timer > 800 && digitalRead(SENSOR_PIN_2) == HIGH) { 
          Serial.println("D2 High Detected! Switching to Part B...");
          state_timer = millis();  
          case2_part_b = true;     
        }
      } 
      else {
        Serial.println("Auto State 2 (Part B): Correcting Left for 0.2s...");
        control_pwm(-AUTO_TURN_PWM, -AUTO_TURN_PWM, 0, 0); 
        
        if (millis() - state_timer >= 250) { 
          Serial.println("0.2s Left Turn Complete. Transition to State 3...");
          state_timer = millis();  
          auto_state = 3;          
        }
      }
      break;

    case 3: 
      Serial.println("Auto State 3: Moving Forward...");
      control_pwm(AUTO_DRIVE_PWM, -AUTO_DRIVE_PWM, 0, 0); 
      
      if (millis() - state_timer > 500) {
        if (digitalRead(SENSOR_PIN_2) == HIGH) {
          Serial.println("D2 High Detected! Transition to State 4...");
          state_timer = millis();  
          auto_state = 4;          
        }
      }
      break; 

    case 4: 
      Serial.println("Auto State 4: Pure Forward...");
      control_pwm(AUTO_DRIVE_PWM, -AUTO_DRIVE_PWM, 0, 0); 
      
      if (millis() - state_timer >= 500) {
        Serial.println("Forward Complete. Transition to State 5 (Turn First)...");
        state_timer = millis();  
        case5_part_b = false; 
        auto_state = 5;          
      }
      break;

    case 5: 
      if (!case5_part_b) {
        Serial.println("Auto State 5 (Part A): Turning Left... Waiting 1500ms");
        control_pwm(-750, -750, 0, 0); 
        
        if (millis() - state_timer >= 800) {
          Serial.println("Left Turn Complete! Switching to Part B (Forward 500ms)...");
          state_timer = millis();  
          case5_part_b = true;     
        }
      } 
      else {
        Serial.println("Auto State 5 (Part B): Moving Forward After Turn... Waiting 500ms");
        control_pwm(AUTO_DRIVE_PWM, -AUTO_DRIVE_PWM, 0, 0); 
        
        if (millis() - state_timer >= 2300) { 
          Serial.println("All Task Complete! Hard Brake.");
          control_pwm(0, 0, 0, 0); 
          auto_state = 6;          
          is_auto_mode = false;    
        }
      }
      break;

    case 6: 
    default:
      control_pwm(0, 0, 0, 0);
      is_auto_mode = false;
      break;
  }
}

// ============ Arduino 初始化 ============
void setup() {
  Serial.begin(115200);
  
  pinMode(SENSOR_PIN_3, INPUT);
  pinMode(SENSOR_PIN_2, INPUT);

  // 設定 A2 腳位為純數位輸出（雷射開關用）
  pinMode(LASER_PIN_A2, OUTPUT);
  digitalWrite(LASER_PIN_A2, LOW); // 開機預設為 LOW (關閉雷射)
  
  myServo3.attach(SERVO_3_PIN);
  myServo3.write(130);

  IIC_Motor_Init(); 
  
  Set_motor_type(MOTOR_TYPE_520);   delay(50); 
  Set_Pluse_Phase(GEAR_RATIO);      delay(50); 
  Set_Pluse_line(PULSE_LINE);       delay(50); 
  Set_Wheel_dis(WHEEL_DIA);         delay(50);

  Set_motor_deadzone(1600);         delay(50); 
  current_deadzone = 1600;
  
  initPS2();
  
  // 🌟 2. 使用大廟控制庫初始化 USB Host Shield 硬體與 ACM 鮑率環境
  Serial.println("Initializing Damiao USB Host Controller...");
  ctrl.init(); 

  // 🌟 3. 套用多載函式使能（Enable）兩個大廟速度馬達實體
  ctrl.enable(motor3_vel); 
  ctrl.enable(motor4_vel); 
}

// ============ 主程式循環 ============
void loop() {
  // 🌟 4. 移除舊的 Usb.Task()，因為大廟函式庫的 ctrl.control_vel() 內部已經自動封裝並處理了 usb_ptr->Task()。
  
  ps2x.read_gamepad(false, 0);
  
  if (ps2x.ButtonPressed(PSB_SQUARE)) { 
    if (!is_auto_mode) {
      Serial.println("SQUARE Pressed: STARTING AUTO PROGRAM!");
      is_auto_mode = true;
      state_timer = millis(); 
      case2_part_b = false;
      case5_part_b = false;
      auto_state = 1;         
    } else {
      Serial.println("SQUARE Pressed: FORCE STOP AUTO PROGRAM!");
      is_auto_mode = false;
      auto_state = 0;
      control_pwm(0, 0, 0, 0); 
    }
  }

  if (is_auto_mode) {
    run_auto_program(); 
  } else {
    control_chassis_joysticks(); 
    
    if(ps2x.Button(PSB_PAD_UP)) {      
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
      control_pwm(MICRO_TURN_PWM, -MICRO_TURN_PWM, 0, 0);
    }
    if(ps2x.Button(PSB_PAD_RIGHT)){
      control_pwm(MICRO_TURN_PWM, MICRO_TURN_PWM, 0, 0);
    }
    if(ps2x.Button(PSB_PAD_LEFT)){
      control_pwm(-MICRO_TURN_PWM, -MICRO_TURN_PWM, 0, 0); 
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
      control_pwm(-MICRO_TURN_PWM, MICRO_TURN_PWM, 0, 0);
    }   
  }

  // 🌟 5. CIRCLE 鍵：使能馬達，並呼叫對應的 control_vel 寫入 RPM 速度
  if(ps2x.NewButtonState(PSB_CIRCLE)) {
    ctrl.enable(motor3_vel);
    ctrl.enable(motor4_vel); 
    
    ctrl.control_vel(motor3_vel, -395.0f); //435RPM
    ctrl.control_vel(motor4_vel, 395.0f); 
  }
  
  // 🌟 6. CROSS 鍵：給予 0 RPM 速度，並將馬達下發失能（Disable）指令
  if(ps2x.NewButtonState(PSB_CROSS)) {
    ctrl.control_vel(motor3_vel, 0.0f); 
    ctrl.control_vel(motor4_vel, 0.0f); 
    
    ctrl.disable(motor3_vel); 
    ctrl.disable(motor4_vel);
  }

  if (ps2x.Button(PSB_TRIANGLE)) {
    myServo3.write(50);
  } else {
    myServo3.write(130);      
  }

  if (ps2x.ButtonPressed(PSB_R1)) {
    laser_state_a2 = !laser_state_a2; // 翻轉開關狀態
    digitalWrite(LASER_PIN_A2, laser_state_a2 ? HIGH : LOW);
  }

  delay(20);
}