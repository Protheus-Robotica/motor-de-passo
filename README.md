# Controle de Motores de Passo

Projeto em desenvolvimento para controle de múltiplos motores de passo em malha aberta com a plataforma do Arduino Mega 2560 fazendo interface com drivers TB6560.

## :rocket: Começando
Para poder começar a utilizar e contribuir no projeto siga esses passos:

- Clone o projeto para sua máquina local
- Siga uma das opções em [Pré-requisitos](#clipboard-pré-requisitos)
- Faça alterações que desejar no código
- Conecte o arduino seguindo as instruções em [Implementação](#package-implementaç)
- Utilize a interface do monitor serial para controle dos motores

## :clipboard: Pré-requisitos
Você pode utilizar qualquer interface que desejar para a utilização do projeto, mas as opções recomendadas são:

### [Arduino-IDE](https://www.arduino.cc/en/software)
> Interface oficial da plataforma arduino para desenvolvimento, para utilizar a mesma copie o código do arquivo [src/main.cpp](./src/main.cpp)
> e cole-o na interface. É a interface ideal para utilizar caso deseje utilizar o projeto de forma mais simples e concisa sem muita configuração.

### [VS-Code](https://code.visualstudio.com/)
> Aliado com a extensão (PlatformIO IDE)[https://platformio.org/platformio-ide] é um ambiente mais robusto e capaz de incluir testes personalizados
> e ter várias informações de desenvolvimento para múltiplas plataformas. Para utilizar é necessário apenas abrir a pasta do projeto no VSCode.

## :package: Implementação
Conecte um Arduino Mega 2560 ao driver de acordo com os pinos definidos pelas diretrizes `#define` no começo do código. Siga o padrão:

`#define MOTOR_OUT_X A` Pino digital `A` deve ser ligado à entrada `CLK+` do driver para o motor X <br>
`#define MOTOR_DIR_X B` Pino digital `B` deve ser ligado à entrada `CW+` do driver para o motor X <br>
`#define MOTOR_EN_X C` Pino digital `C` deve ser ligado à entrada `EN+` do driver para o motor X <br>

Os pinos `CLK-`, `CW-` e `EN-` devem todos serem interligados no `GND` do arduino e da fonte.

## :bust_in_silhouette: Contribuidores
- [Gabriel Soares](https://github.com/Gvinfinity)

## :page_with_curl: Licença
Este projeto está sob a licença MIT - Mais informações em: [LICENSE.md](./LICENSE.md)
