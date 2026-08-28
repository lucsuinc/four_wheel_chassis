//
// Created by 34343 on 2026/8/28.
//

//zdt步进电机的通信协议


#include "Emm_v5.h"

 uint8_t cmd[64] = {0};
/**
  * @brief    灏嗗綋鍓嶄綅缃竻闆?
  * @param    addr  锛氱數鏈哄湴鍧€
  * @retval   鍦板潃 + 鍔熻兘鐮?+ 鍛戒护鐘舵€?+ 鏍￠獙瀛楄妭
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{

  // 瑁呰浇鍛戒护
  cmd[0] =  addr;                       // 鍦板潃
  cmd[1] =  0x0A;                       // 鍔熻兘鐮?
  cmd[2] =  0x6D;                       // 杈呭姪鐮?
  cmd[3] =  0x6B;                       // 鏍￠獙瀛楄妭

  // 鍙戦€佸懡浠?
  HAL_UART_Transmit(&huart1, cmd, 4, HAL_MAX_DELAY);
}

/**
  * @brief    瑙ｉ櫎鍫佃浆淇濇姢
  * @param    addr  锛氱數鏈哄湴鍧€
  * @retval   鍦板潃 + 鍔熻兘鐮?+ 鍛戒护鐘舵€?+ 鏍￠獙瀛楄妭
  */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{

  // 瑁呰浇鍛戒护
  cmd[0] =  addr;                       // 鍦板潃
  cmd[1] =  0x0E;                       // 鍔熻兘鐮?
  cmd[2] =  0x52;                       // 杈呭姪鐮?
  cmd[3] =  0x6B;                       // 鏍￠獙瀛楄妭

  // 鍙戦€佸懡浠?
   HAL_UART_Transmit(&huart1, cmd, 4, HAL_MAX_DELAY);
}

/**
  * @brief    璇诲彇绯荤粺鍙傛暟
  * @param    addr  锛氱數鏈哄湴鍧€
  * @param    s     锛氱郴缁熷弬鏁扮被鍨?
  * @retval   鍦板潃 + 鍔熻兘鐮?+ 鍛戒护鐘舵€?+ 鏍￠獙瀛楄妭
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;

  // 瑁呰浇鍛戒护
  cmd[i] = addr; ++i;                   // 鍦板潃

  switch(s)                             // 鍔熻兘鐮?
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 鏍￠獙瀛楄妭

  // 鍙戦€佸懡浠?
   HAL_UART_Transmit(&huart1, cmd, i, HAL_MAX_DELAY);
}

/**
  * @brief    淇敼寮€鐜?闂幆鎺у埗妯″紡
  * @param    addr     锛氱數鏈哄湴鍧€
  * @param    svF      锛氭槸鍚﹀瓨鍌ㄦ爣蹇楋紝false涓轰笉瀛樺偍锛宼rue涓哄瓨鍌?
  * @param    ctrl_mode锛氭帶鍒舵ā寮忥紙瀵瑰簲灞忓箷涓婄殑P_Pul鑿滃崟锛夛紝0鏄叧闂剦鍐茶緭鍏ュ紩鑴氾紝1鏄紑鐜ā寮忥紝2鏄棴鐜ā寮忥紝3鏄En绔彛澶嶇敤涓哄鍦堥檺浣嶅紑鍏宠緭鍏ュ紩鑴氾紝Dir绔彛澶嶇敤涓哄埌浣嶈緭鍑洪珮鐢靛钩鍔熻兘
  * @retval   鍦板潃 + 鍔熻兘鐮?+ 鍛戒护鐘舵€?+ 鏍￠獙瀛楄妭
  */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{

  // 瑁呰浇鍛戒护
  cmd[0] =  addr;                       // 鍦板潃
  cmd[1] =  0x46;                       // 鍔熻兘鐮?
  cmd[2] =  0x69;                       // 杈呭姪鐮?
  cmd[3] =  svF;                        // 鏄惁瀛樺偍鏍囧織锛宖alse涓轰笉瀛樺偍锛宼rue涓哄瓨鍌?
  cmd[4] =  ctrl_mode;                  // 鎺у埗妯″紡锛堝搴斿睆骞曚笂鐨凱_Pul鑿滃崟锛夛紝0鏄叧闂剦鍐茶緭鍏ュ紩鑴氾紝1鏄紑鐜ā寮忥紝2鏄棴鐜ā寮忥紝3鏄En绔彛澶嶇敤涓哄鍦堥檺浣嶅紑鍏宠緭鍏ュ紩鑴氾紝Dir绔彛澶嶇敤涓哄埌浣嶈緭鍑洪珮鐢靛钩鍔熻兘
  cmd[5] =  0x6B;                       // 鏍￠獙瀛楄妭

  // 鍙戦€佸懡浠?
   HAL_UART_Transmit(&huart1, cmd, 6, HAL_MAX_DELAY);
}

/**
  * @brief    浣胯兘淇″彿鎺у埗
  * @param    addr  锛氱數鏈哄湴鍧€
  * @param    state 锛氫娇鑳界姸鎬?    锛宼rue涓轰娇鑳界數鏈猴紝false涓哄叧闂數鏈?
  * @param    snF   锛氬鏈哄悓姝ユ爣蹇?锛宖alse涓轰笉鍚敤锛宼rue涓哄惎鐢?
  * @retval   鍦板潃 + 鍔熻兘鐮?+ 鍛戒护鐘舵€?+ 鏍￠獙瀛楄妭
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{

  // 瑁呰浇鍛戒护
  cmd[0] =  addr;                       // 鍦板潃
  cmd[1] =  0xF3;                       // 鍔熻兘鐮?
  cmd[2] =  0xAB;                       // 杈呭姪鐮?
  cmd[3] =  (uint8_t)state;             // 浣胯兘鐘舵€?
  cmd[4] =  snF;                        // 澶氭満鍚屾杩愬姩鏍囧織
  cmd[5] =  0x6B;                       // 鏍￠獙瀛楄妭

  // 鍙戦€佸懡浠?
   HAL_UART_Transmit(&huart1, cmd, 6, HAL_MAX_DELAY);
}

/**
  * @brief    閫熷害妯″紡
  * @param    addr锛氱數鏈哄湴鍧€
  * @param    dir 锛氭柟鍚?      锛?涓篊W锛屽叾浣欏€间负CCW
  * @param    vel 锛氶€熷害       锛岃寖鍥? - 5000RPM
  * @param    acc 锛氬姞閫熷害     锛岃寖鍥? - 255锛屾敞鎰忥細0鏄洿鎺ュ惎鍔?
  * @param    snF 锛氬鏈哄悓姝ユ爣蹇楋紝false涓轰笉鍚敤锛宼rue涓哄惎鐢?
  * @retval   鍦板潃 + 鍔熻兘鐮?+ 鍛戒护鐘舵€?+ 鏍￠獙瀛楄妭
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{

  // 瑁呰浇鍛戒护
  cmd[0] =  addr;                       // 鍦板潃
  cmd[1] =  0xF6;                       // 鍔熻兘鐮?
  cmd[2] =  dir;                        // 鏂瑰悜
  cmd[3] =  (uint8_t)(vel >> 8);        // 閫熷害(RPM)楂?浣嶅瓧鑺?
  cmd[4] =  (uint8_t)(vel >> 0);        // 閫熷害(RPM)浣?浣嶅瓧鑺?
  cmd[5] =  acc;                        // 鍔犻€熷害锛屾敞鎰忥細0鏄洿鎺ュ惎鍔?
  cmd[6] =  snF;                        // 澶氭満鍚屾杩愬姩鏍囧織
  cmd[7] =  0x6B;                       // 鏍￠獙瀛楄妭

  // 鍙戦€佸懡浠?
   HAL_UART_Transmit(&huart1, cmd, 8, HAL_MAX_DELAY);
}

/**
  * @brief    浣嶇疆妯″紡
  * @param    addr锛氱數鏈哄湴鍧€
  * @param    dir 锛氭柟鍚?       锛?涓篊W锛屽叾浣欏€间负CCW
  * @param    vel 锛氶€熷害(RPM)   锛岃寖鍥? - 5000RPM
  * @param    acc 锛氬姞閫熷害      锛岃寖鍥? - 255锛屾敞鎰忥細0鏄洿鎺ュ惎鍔?
  * @param    clk 锛氳剦鍐叉暟      锛岃寖鍥?- (2^32 - 1)涓?
  * @param    raF 锛氱浉浣?缁濆鏍囧織锛宖alse涓虹浉瀵硅繍鍔紝true涓虹粷瀵瑰€艰繍鍔?
  * @param    snF 锛氬鏈哄悓姝ユ爣蹇?锛宖alse涓轰笉鍚敤锛宼rue涓哄惎鐢?
  * @retval   鍦板潃 + 鍔熻兘鐮?+ 鍛戒护鐘舵€?+ 鏍￠獙瀛楄妭
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{

  // 瑁呰浇鍛戒护
  cmd[0]  =  addr;                      // 鍦板潃
  cmd[1]  =  0xFD;                      // 鍔熻兘鐮?
  cmd[2]  =  dir;                       // 鏂瑰悜
  cmd[3]  =  (uint8_t)(vel >> 8);       // 閫熷害(RPM)楂?浣嶅瓧鑺?
  cmd[4]  =  (uint8_t)(vel >> 0);       // 閫熷害(RPM)浣?浣嶅瓧鑺?
  cmd[5]  =  acc;                       // 鍔犻€熷害锛屾敞鎰忥細0鏄洿鎺ュ惎鍔?
  cmd[6]  =  (uint8_t)(clk >> 24);      // 鑴夊啿鏁?bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 鑴夊啿鏁?bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 鑴夊啿鏁?bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 鑴夊啿鏁?bit0  - bit7 )
  cmd[10] =  raF;                       // 鐩镐綅/缁濆鏍囧織锛宖alse涓虹浉瀵硅繍鍔紝true涓虹粷瀵瑰€艰繍鍔?
  cmd[11] =  snF;                       // 澶氭満鍚屾杩愬姩鏍囧織锛宖alse涓轰笉鍚敤锛宼rue涓哄惎鐢?
  cmd[12] =  0x6B;                      // 鏍￠獙瀛楄妭

  // 鍙戦€佸懡浠?
   HAL_UART_Transmit(&huart1, cmd, 13, HAL_MAX_DELAY);
}

/**
  * @brief    绔嬪嵆鍋滄锛堟墍鏈夋帶鍒舵ā寮忛兘閫氱敤锛?
  * @param    addr  锛氱數鏈哄湴鍧€
  * @param    snF   锛氬鏈哄悓姝ユ爣蹇楋紝false涓轰笉鍚敤锛宼rue涓哄惎鐢?
  * @retval   鍦板潃 + 鍔熻兘鐮?+ 鍛戒护鐘舵€?+ 鏍￠獙瀛楄妭
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{

  // 瑁呰浇鍛戒护
  cmd[0] =  addr;                       // 鍦板潃
  cmd[1] =  0xFE;                       // 鍔熻兘鐮?
  cmd[2] =  0x98;                       // 杈呭姪鐮?
  cmd[3] =  snF;                        // 澶氭満鍚屾杩愬姩鏍囧織
  cmd[4] =  0x6B;                       // 鏍￠獙瀛楄妭

  // 鍙戦€佸懡浠?
   HAL_UART_Transmit(&huart1, cmd, 5, HAL_MAX_DELAY);
}

/**
  * @brief    澶氭満鍚屾杩愬姩
  * @param    addr  锛氱數鏈哄湴鍧€
  * @retval   鍦板潃 + 鍔熻兘鐮?+ 鍛戒护鐘舵€?+ 鏍￠獙瀛楄妭
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{

  // 瑁呰浇鍛戒护
  cmd[0] =  addr;                       // 鍦板潃
  cmd[1] =  0xFF;                       // 鍔熻兘鐮?
  cmd[2] =  0x66;                       // 杈呭姪鐮?
  cmd[3] =  0x6B;                       // 鏍￠獙瀛楄妭

  // 鍙戦€佸懡浠?
   HAL_UART_Transmit(&huart1, cmd, 4, HAL_MAX_DELAY);
}

/**
  * @brief    璁剧疆鍗曞湀鍥為浂鐨勯浂鐐逛綅缃?
  * @param    addr  锛氱數鏈哄湴鍧€
  * @param    svF   锛氭槸鍚﹀瓨鍌ㄦ爣蹇楋紝false涓轰笉瀛樺偍锛宼rue涓哄瓨鍌?
  * @retval   鍦板潃 + 鍔熻兘鐮?+ 鍛戒护鐘舵€?+ 鏍￠獙瀛楄妭
  */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{

  // 瑁呰浇鍛戒护
  cmd[0] =  addr;                       // 鍦板潃
  cmd[1] =  0x93;                       // 鍔熻兘鐮?
  cmd[2] =  0x88;                       // 杈呭姪鐮?
  cmd[3] =  svF;                        // 鏄惁瀛樺偍鏍囧織锛宖alse涓轰笉瀛樺偍锛宼rue涓哄瓨鍌?
  cmd[4] =  0x6B;                       // 鏍￠獙瀛楄妭

  // 鍙戦€佸懡浠?
   HAL_UART_Transmit(&huart1, cmd, 5, HAL_MAX_DELAY);
}

/**
  * @brief    淇敼鍥為浂鍙傛暟
  * @param    addr  锛氱數鏈哄湴鍧€
  * @param    svF   锛氭槸鍚﹀瓨鍌ㄦ爣蹇楋紝false涓轰笉瀛樺偍锛宼rue涓哄瓨鍌?
  * @param    o_mode 锛氬洖闆舵ā寮忥紝0涓哄崟鍦堝氨杩戝洖闆讹紝1涓哄崟鍦堟柟鍚戝洖闆讹紝2涓哄鍦堟棤闄愪綅纰版挒鍥為浂锛?涓哄鍦堟湁闄愪綅寮€鍏冲洖闆?
  * @param    o_dir  锛氬洖闆舵柟鍚戯紝0涓篊W锛屽叾浣欏€间负CCW
  * @param    o_vel  锛氬洖闆堕€熷害锛屽崟浣嶏細RPM锛堣浆/鍒嗛挓锛?
  * @param    o_tm   锛氬洖闆惰秴鏃舵椂闂达紝鍗曚綅锛氭绉?
  * @param    sl_vel 锛氭棤闄愪綅纰版挒鍥為浂妫€娴嬭浆閫燂紝鍗曚綅锛歊PM锛堣浆/鍒嗛挓锛?
  * @param    sl_ma  锛氭棤闄愪綅纰版挒鍥為浂妫€娴嬬數娴侊紝鍗曚綅锛歁a锛堟瀹夛級
  * @param    sl_ms  锛氭棤闄愪綅纰版挒鍥為浂妫€娴嬫椂闂达紝鍗曚綅锛歁s锛堟绉掞級
  * @param    potF   锛氫笂鐢佃嚜鍔ㄨЕ鍙戝洖闆讹紝false涓轰笉浣胯兘锛宼rue涓轰娇鑳?
  * @retval   鍦板潃 + 鍔熻兘鐮?+ 鍛戒护鐘舵€?+ 鏍￠獙瀛楄妭
  */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{

  // 瑁呰浇鍛戒护
  cmd[0] =  addr;                       // 鍦板潃
  cmd[1] =  0x4C;                       // 鍔熻兘鐮?
  cmd[2] =  0xAE;                       // 杈呭姪鐮?
  cmd[3] =  svF;                        // 鏄惁瀛樺偍鏍囧織锛宖alse涓轰笉瀛樺偍锛宼rue涓哄瓨鍌?
  cmd[4] =  o_mode;                     // 鍥為浂妯″紡锛?涓哄崟鍦堝氨杩戝洖闆讹紝1涓哄崟鍦堟柟鍚戝洖闆讹紝2涓哄鍦堟棤闄愪綅纰版挒鍥為浂锛?涓哄鍦堟湁闄愪綅寮€鍏冲洖闆?
  cmd[5] =  o_dir;                      // 鍥為浂鏂瑰悜
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 鍥為浂閫熷害(RPM)楂?浣嶅瓧鑺?
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 鍥為浂閫熷害(RPM)浣?浣嶅瓧鑺?
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 鍥為浂瓒呮椂鏃堕棿(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 鍥為浂瓒呮椂鏃堕棿(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 鍥為浂瓒呮椂鏃堕棿(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 鍥為浂瓒呮椂鏃堕棿(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 鏃犻檺浣嶇鎾炲洖闆舵娴嬭浆閫?RPM)楂?浣嶅瓧鑺?
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 鏃犻檺浣嶇鎾炲洖闆舵娴嬭浆閫?RPM)浣?浣嶅瓧鑺?
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 鏃犻檺浣嶇鎾炲洖闆舵娴嬬數娴?Ma)楂?浣嶅瓧鑺?
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 鏃犻檺浣嶇鎾炲洖闆舵娴嬬數娴?Ma)浣?浣嶅瓧鑺?
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 鏃犻檺浣嶇鎾炲洖闆舵娴嬫椂闂?Ms)楂?浣嶅瓧鑺?
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 鏃犻檺浣嶇鎾炲洖闆舵娴嬫椂闂?Ms)浣?浣嶅瓧鑺?
  cmd[18] =  potF;                      // 涓婄數鑷姩瑙﹀彂鍥為浂锛宖alse涓轰笉浣胯兘锛宼rue涓轰娇鑳?
  cmd[19] =  0x6B;                      // 鏍￠獙瀛楄妭

  // 鍙戦€佸懡浠?
   HAL_UART_Transmit(&huart1, cmd, 20, HAL_MAX_DELAY);
}

/**
  * @brief    瑙﹀彂鍥為浂
  * @param    addr   锛氱數鏈哄湴鍧€
  * @param    o_mode 锛氬洖闆舵ā寮忥紝0涓哄崟鍦堝氨杩戝洖闆讹紝1涓哄崟鍦堟柟鍚戝洖闆讹紝2涓哄鍦堟棤闄愪綅纰版挒鍥為浂锛?涓哄鍦堟湁闄愪綅寮€鍏冲洖闆?
  * @param    snF   锛氬鏈哄悓姝ユ爣蹇楋紝false涓轰笉鍚敤锛宼rue涓哄惎鐢?
  * @retval   鍦板潃 + 鍔熻兘鐮?+ 鍛戒护鐘舵€?+ 鏍￠獙瀛楄妭
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{

  // 瑁呰浇鍛戒护
  cmd[0] =  addr;                       // 鍦板潃
  cmd[1] =  0x9A;                       // 鍔熻兘鐮?
  cmd[2] =  o_mode;                     // 鍥為浂妯″紡锛?涓哄崟鍦堝氨杩戝洖闆讹紝1涓哄崟鍦堟柟鍚戝洖闆讹紝2涓哄鍦堟棤闄愪綅纰版挒鍥為浂锛?涓哄鍦堟湁闄愪綅寮€鍏冲洖闆?
  cmd[3] =  snF;                        // 澶氭満鍚屾杩愬姩鏍囧織锛宖alse涓轰笉鍚敤锛宼rue涓哄惎鐢?
  cmd[4] =  0x6B;                       // 鏍￠獙瀛楄妭

  // 鍙戦€佸懡浠?
   HAL_UART_Transmit(&huart1, cmd, 5, HAL_MAX_DELAY);
}

/**
  * @brief    寮哄埗涓柇骞堕€€鍑哄洖闆?
  * @param    addr  锛氱數鏈哄湴鍧€
  * @retval   鍦板潃 + 鍔熻兘鐮?+ 鍛戒护鐘舵€?+ 鏍￠獙瀛楄妭
  */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{

  // 瑁呰浇鍛戒护
  cmd[0] =  addr;                       // 鍦板潃
  cmd[1] =  0x9C;                       // 鍔熻兘鐮?
  cmd[2] =  0x48;                       // 杈呭姪鐮?
  cmd[3] =  0x6B;                       // 鏍￠獙瀛楄妭

  // 鍙戦€佸懡浠?
   HAL_UART_Transmit(&huart1, cmd, 4, HAL_MAX_DELAY);
}

