# Connect 4 Playing Robot (Ongoing)

**This project is currently under active development.**

An autonomous robot designed to play Connect 4 against a human opponent using piece sensing, game AI, and electromechanical actuation. 

## Project Goals
- Detect and map Connect 4 board state using sensors
- Implement game-playing AI using minimax algorithm and heuristics
- Physical PID actuation to place game pieces
- Enable human-vs-robot interface and gameplay

## Current Progress
**Completed:** 
- Mechanical design concept and prototyping
- Game logic and bitboard representation
- AI minimax algorithm

**In Progress:**
- IR sensor circuitry for move detection
- PID control for physical actuation
- Minimax heuristic NES optimization

**Planned Work:**
- Design piece magazine and automatic loader for fully autonomous gameplay
- Replace Arduino with stronger microcontroller
- Custom PCB design
- Full system integration and gameplay testing

## Code Structure
- `src/arduino/` - Embedded Arduino C++ code for motor control and reading sensors
- `src/game_ai/` - Python-based Connect 4 game logic and minimax AI, including NES weight optimization algorithm

## Media
_Mechanical prototype. Click for video demo (11/13/2025)_

[![Piece dropping prototyping](media/connect-4-prototype-thumbnail.jpg)](https://youtu.be/RXw5a7y7fcY)

_Sensor circuitry prototyping (11/19/2025)_

![Sensor prototyping photo](media/sensors-prototyping.jpg)

## Status 
**Active development** - updated regularly. 
