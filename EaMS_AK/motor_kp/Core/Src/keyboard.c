/*
 * keyboard.c
 *
 *  Created on: Oct 4, 2024
 *      Author: MSI
 */


#include "main.h"

int keyboard_process();
// Порты, к которым подключена клавиатура
GPIO_TypeDef* Keypad_port[2][4] = {{Keyboard_row1_GPIO_Port,Keyboard_row2_GPIO_Port,Keyboard_row3_GPIO_Port,Keyboard_row4_GPIO_Port}, {Keyboard_col1_GPIO_Port,Keyboard_col2_GPIO_Port,Keyboard_col3_GPIO_Port,Keyboard_col4_GPIO_Port}};
// Номера контактов, к которым подключена клавиатура
uint16_t Keypad_pin[2][4] = {{Keyboard_row1_Pin,Keyboard_row2_Pin,Keyboard_row3_Pin,Keyboard_row4_Pin},{Keyboard_col1_Pin,Keyboard_col2_Pin,Keyboard_col3_Pin,Keyboard_col4_Pin}};
// Значения кнопок на клавиатуре
int keys[4][4] = {
 {1, 2, 3, 10},
 {4, 5, 6, 11},
 {7, 8, 9, 12},
 {0, 15, 14, 13}
};
int key; // Кнопка клавиатуры
int num1 = -1; // Первый аргумент



// Перевести клавиатуру в исходное состояние
  void reset_key_pins(){
  for(int i = 0; i<4; i++){
  HAL_GPIO_WritePin(Keypad_port[1][i],Keypad_pin[1][i],0);
  }
  }
  // Проверка, какая кнопка сейчас нажата. Если не нажата, то возвращает -1
  int read_key(){
  int col_num = -1;
  int row_num = -1;
  for(int i = 0; i<4; i++){
  // Ищем строчку, которая замкнулась на землю
  if(!HAL_GPIO_ReadPin(Keypad_port[0][i],Keypad_pin[0][i])){
  row_num = i;
  }
  }
  // Если найдена строка с нажатой кнопкой
  if(row_num != -1){
  for(int i = 0; i<4; i++){
  // По очереди подаём логич. 1 на разные столбцы и отслеживаем
  // на каком значение строки тоже установится в 1
  HAL_GPIO_WritePin(Keypad_port[1][i],Keypad_pin[1][i],1);

  if(col_num == -1 && HAL_GPIO_ReadPin(Keypad_port[0][row_num],
  Keypad_pin[0][row_num])){
  col_num = i;
  }
  }
  // Обратно замыкаем все контакты столбцов на землю
  reset_key_pins();
  if(col_num!=-1){
  // По номерам строки и столбца определяем число
  int curKey = keys[row_num][col_num];
  return curKey;
  }
  // Найдена строка, а столбец нет
  return -1;
  }
  // Ни одна из кнопок не нажата
  return -1;
  }


  int keyboard_process(){
	  key = read_key();
	  if(key!=-1){ // Если кнопка нажата
	  num1 = key;
	  }
	  HAL_Delay(200);
	  return num1;
  }

