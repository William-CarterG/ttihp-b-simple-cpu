# Computadora FPGA de Arquitectura Harvard

## Descripción General

Este proyecto implementa una computadora educativa de 8 bits con arquitectura Harvard en una placa FPGA Go-board. La computadora incluye todos los componentes esenciales de una CPU funcional y demuestra los principios fundamentales de la arquitectura de computadoras.

## Características Principales

- **Arquitectura Harvard**: Memorias de instrucciones y datos separadas
- **Ruta de datos de 8 bits**: Todos los registros y operaciones utilizan valores de 8 bits
- **Ciclo de ejecución completo**: Búsqueda, decodificación, ejecución y actualización
- **Memoria de instrucciones programable**: Programa codificado que demuestra varias operaciones

## Componentes del Sistema

La computadora incluye los siguientes componentes:

- Contador de Programa (PC)
- Memoria de Instrucciones
- Unidad de Control
- Registros A y B de propósito general
- Unidad Aritmética Lógica (ALU)
- Registro de Estado
- Memoria de Datos
- Multiplexores para el enrutamiento de datos

## Conjunto de Instrucciones

El sistema soporta varias operaciones:

- Movimiento de datos (carga, almacenamiento)
- Operaciones aritméticas (suma, resta)
- Operaciones lógicas (AND, OR, NOT, XOR)
- Operaciones de desplazamiento (izquierda, derecha)
- Control de flujo (saltos condicionales e incondicionales)

## Cómo Usar la Computadora FPGA

### Requisitos

- Placa FPGA Go-board
- Cable Micro USB
- Software [Apio](https://github.com/FPGAwars/apio?tab=readme-ov-file) instalado

### Instrucciones de Instalación

1. Conecta tu placa Go-board a tu computadora mediante USB
2. Navega al directorio del proyecto:
   ```
   cd ruta/a/ttihp-b-simple-cpu/src/go-board/template
   ```
3. Compila el proyecto:
   ```
   apio build
   ```
4. Carga el programa en la FPGA:
   ```
   apio upload
   ```

### Controles de la Placa

- **SW1 (Botón de Reset)**: Presiona para reiniciar la CPU
- **SW2 (Botón de Paso)**: Presiona para ejecutar una instrucción a la vez
- **SW3 (Modo de Visualización)**: Presiona para cambiar entre diferentes modos de visualización:
  - Modo 0: Muestra los 4 bits inferiores del Contador de Programa
  - Modo 1: Muestra los 4 bits inferiores del Registro A
  - Modo 2: Muestra los 4 bits inferiores del Registro B
  - Modo 3: Muestra los 4 bits inferiores del resultado de la ALU
- **SW4**: Botón adicional para funcionalidades futuras

### Interpretación de los LEDs

Los LEDs muestran información según el modo de visualización seleccionado:
- LED1 (izquierda): Bit más significativo (MSB)
- LED4 (derecha): Bit menos significativo (LSB)

## Modificación del Programa

Para modificar el programa ejecutado por la CPU, edita la sección `instruction_memory` en el archivo `fpga_computer.v`. Cada instrucción está documentada con comentarios que muestran los valores de los registros después de su ejecución.

## Valor Educativo

Esta computadora demuestra conceptos fundamentales de arquitectura de computadoras, permitiendo observar el estado interno de la CPU mientras ejecuta instrucciones. Es una herramienta valiosa para comprender cómo funcionan las computadoras a nivel básico.

---

Desarrollado como parte del proyecto educativo de arquitectura de computadoras.
