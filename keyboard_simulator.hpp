#pragma once

#ifndef KEYBOARD_SIMULATOR_H
#define KEYBOARD_SIMULATOR_H

#define KEY_O_SCANCODE 0x18
#define KEY_9_SCANCODE 0x0A
#define KEY_A_SCANCODE 0x1E
#define KEY_JUMP_SCANCODE 0x148
#define KEY_DUCK_SCANCODE 0x150
#define KEY_BACK_SCANCODE 0x14B
#define KEY_FORWARD_SCANCODE 0x14D

#include <windows.h>
#include <stdint.h>

class KeyboardSimulator {
public:
	void forward();
	void backward();
	void duck();
	void jump();
	void kick();
	void punch();

	//special moves
	void soryogin();
private:
	void send_scan_code(const char& scan_code, uint32_t sleep_time);

};

#endif // !1