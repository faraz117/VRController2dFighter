#include "keyboard_simulator.hpp"

void KeyboardSimulator::send_scan_code(const char& scan_code, uint32_t sleep_time) {
	INPUT KEY;

	KEY.type = INPUT_KEYBOARD;
	KEY.ki.time = 0;
	KEY.ki.wVk = 0;
	KEY.ki.dwExtraInfo = 0;
	KEY.ki.dwFlags = KEYEVENTF_SCANCODE;
	KEY.ki.wScan = scan_code;
	SendInput(1, &KEY, sizeof(INPUT));

	Sleep((sleep_time));

	KEY.ki.dwFlags = KEYEVENTF_SCANCODE | KEYEVENTF_KEYUP;
	SendInput(1, &KEY, sizeof(INPUT));
}

void KeyboardSimulator::forward()
{
}

void KeyboardSimulator::backward()
{
}

void KeyboardSimulator::jump()
{
}


void KeyboardSimulator::duck() {
	send_scan_code(KEY_DUCK_SCANCODE, 10);
}

void KeyboardSimulator::kick()
{
	send_scan_code(KEY_9_SCANCODE, 10);
}

void KeyboardSimulator::punch()
{
	send_scan_code(KEY_O_SCANCODE, 10);
}

void KeyboardSimulator::soryogin() {

	send_scan_code(KEY_DUCK_SCANCODE, 1);
	INPUT KEY;

	// Pressing two keys together
	KEY.type = INPUT_KEYBOARD;
	KEY.ki.time = 0;
	KEY.ki.wVk = 0;
	KEY.ki.dwExtraInfo = 0;
	KEY.ki.dwFlags = KEYEVENTF_SCANCODE;
	KEY.ki.wScan = KEY_DUCK_SCANCODE;
	SendInput(1, &KEY, sizeof(INPUT));
	KEY.ki.wScan = KEY_FORWARD_SCANCODE;
	SendInput(1, &KEY, sizeof(INPUT));
	Sleep((1));

	KEY.ki.dwFlags = KEYEVENTF_SCANCODE | KEYEVENTF_KEYUP;
	SendInput(1, &KEY, sizeof(INPUT));
	KEY.ki.wScan = KEY_DUCK_SCANCODE;
	KEY.ki.dwFlags = KEYEVENTF_SCANCODE | KEYEVENTF_KEYUP;
	SendInput(1, &KEY, sizeof(INPUT));

	send_scan_code(KEY_FORWARD_SCANCODE, 1);

	punch();

}
