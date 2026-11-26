# V3S_Stepper_Firmware
Firmware do robô VSSS V3S com motores de passo, para a placa **V3s stepper** (ESP32-C3 SuperMini + 2× DRV8825 + GY-521).

> Baseado no branch `esp_now` de [iOsnaaente/VSSS_Stepper](https://github.com/iOsnaaente/VSSS_Stepper) (commit `6944a05`), com o histórico preservado.
> A GUI de testes (com keepalive de velocidade) fica no branch `icaro_no_ramp` do repositório privado `TauraBots/V3S_IDF_32_Firmware` e ainda não foi trazida para cá.

## 🔌 Hardware / Pinagem
Placa de duas camadas (superior com o ESP32-C3, inferior com os drivers), ligadas por um conector 2×6. Pinagem conferida com o esquemático `V3s.kicad_sch`:

| ESP32-C3 | Sinal | Destino |
|----------|-------|---------|
| GPIO0 | `M1_VEL_PIN` (STEP_1) | DRV8825 #1 STEP |
| GPIO4 | `M1_DIR_PIN` (DIR_1) | DRV8825 #1 DIR |
| GPIO1 | `M2_VEL_PIN` (STEP_2) | DRV8825 #2 STEP |
| GPIO5 | `M2_DIR_PIN` (DIR_2) | DRV8825 #2 DIR |
| GPIO10 | `ENABLE_PIN` | ~EN dos dois drivers (ativo em nível baixo) |
| GPIO8 / GPIO9 | I2C SDA / SCL | GY-521 (MPU6050), OLED, conector I2C aux |
| GPIO3 | Bat_S | Sensor de bateria (divisor ainda não montado) |

- Microstep: jumpers M0–M2 em 3V3 por padrão → **1/32**.
- ~SLP ligado em 3V3; ~RST com pull-up e botão de reset.

## 🔧 Build
PlatformIO, ambiente `esp32c3-supermini` (Arduino). Transporte padrão: ESP-NOW no canal 6 (`-DUSE_ESPNOW_TRANSPORT`, `-DESPNOW_CHANNEL=6` em `platformio.ini`).

```
pio run -e esp32c3-supermini -t upload
```

---

## Notas originais (PWM_Stepper)
Controle de motores de passo usando bloco de comando PWM dentro do ESP32.

## 📡 Conectividade (WiFi + UDP)
O firmware conecta-se a uma rede WiFi (modo Station) e escuta comandos de controle via UDP.

### Arquivo de Credenciais
Para evitar expor SSID/Senha no repositório, foi adicionado um mecanismo de template:

1. Copie o arquivo `include/credentials.template.h` para `include/credentials.h`:
	 ```
	 cp include/credentials.template.h include/credentials.h
	 ```
2. Edite `include/credentials.h` e preencha:
	 - `WIFI_SSID`
	 - `WIFI_PASSWORD`
3. Compile e faça o upload normalmente com o PlatformIO.

> O arquivo `include/credentials.h` está listado no `.gitignore` e não será versionado.

### Macros Principais de Configuração
| Macro | Função |
|-------|--------|
| `WIFI_SSID` | Nome da rede WiFi para conexão |
| `WIFI_PASSWORD` | Senha da rede WiFi |
| `UDP_LISTEN_PORT` | Porta UDP onde o firmware escuta comandos |
| `UDP_MAX_PACKET_SIZE` | Tamanho máximo de pacote UDP aceito |

### Formatos de Pacote de Controle
O listener UDP aceita múltiplos formatos para dirigir as rodas independentemente:
1. Binário compacto: 2 bytes (L, R) normalizados em `int8_t` (–127..127) + opcional checksum XOR (3º byte)
2. ASCII CSV: `"<left>,<right>"` (ex: `"0.5,-0.25"`)
3. Legado 1 byte: `F`,`B`,`L`,`R` ou qualquer outro (stop)

### Segurança Básica
Para evitar acessos indevidos:
- Use uma VLAN ou rede isolada em ambientes críticos.
- Avalie adicionar autenticação no protocolo caso seja exposto fora de rede local.

## 🛠 Debug Serial
Use as macros `DEBUG_SERIAL` para mensagens. Modos disponíveis em `board_config.h`:
```
// #define DEBUG_SERIAL_COMPLETO
#define DEBUG_SERIAL_REDUZIDO
// #define DEBUG_SERIAL_DESLIGADO
```

## ⚙ Motores de Passo
Cada motor é controlado via PWM usando o periférico LEDC. A classe `Stepper` permite:
- Ajuste de velocidade normalizada (−1.0 a 1.0)
- Controle de torque (enable/disable)
- Conversão de velocidade para frequência de passos

## 🤖 Classe Robot
Abstrai dois motores (esquerdo/direito) e fornece:
- `drive(vel, turn)` mistura diferencial
- `drive_wheels(left, right)` controle direto
- `stop()`

## 📂 Estrutura do Projeto
```
include/
	credentials.template.h  <- Template de credenciais
	credentials.h           <- Sua cópia (não versionada)
src/
	main.cpp                <- Inicialização WiFi + loop de status + tarefa UDP
	Robot/                  <- Lógica de alto nível
	Stepper/                <- Driver simplificado de motor
	Serial/                 <- Infra de debug
board_config.h            <- Pinos e includes globais
platformio.ini            <- Configuração PlatformIO
```

## 🚀 Próximos Passos Sugeridos
- Implementar autenticação leve nos pacotes (ex: token de 1 byte + checksum)
- Adicionar OTA (já há comentários no `platformio.ini` para espota)
- Criar script Python para enviar comandos binários/CSV facilmente

## 📄 Licença
Adicione aqui a licença do projeto (ex: MIT, Apache 2.0, etc.).

