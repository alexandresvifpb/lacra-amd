#include <Arduino.h>
#include "Wire.h"
#include "SSD1306.h"
 
#define OLED_PIN_SDA            (4)
#define OLED_PIN_SCL            (15)

SSD1306 display(0x3c, OLED_PIN_SDA, OLED_PIN_SCL);

int contador;

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println();
 
    // Inicializa o objeto que controlará o que será exibido na tela
    display.init();
 
    //gira o display 180º (deixa de ponta cabeça)
    // display.flipScreenVertically();
 
    //configura a fonte de escrita "ArialMT_Plain_10"
    display.setFont(ArialMT_Plain_10);
}

void loop() {
    contador++;

    //limpa todo o display, apaga o contúdo da tela
    display.clear();

    display.drawString(0, 0, String(contador));

    //exibe na tela o que foi configurado até então.
    display.display();

    delay(1000);
}