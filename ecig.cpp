#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <iostream>
#include "wiringPi/wiringPi/wiringPi.h"
#include "wiringPi/wiringPi/wiringPiSPI.h"
#include <time.h>
#include "ABE_ADCPi.h"

float resistance = 0.0;
float voltage = 0.0;
float current = 0.0;
int power = 0;
int temperature = 0;
float bias = 1.0;
int output_pwm = 0;
int enc_clk_last_state;
int low_batt_alarm = 0;
int high_temp = 0;
int status = 0;
int computed_feedback = 0;
long long temp_write_time = 0;
long long backlight_time = 0;
int digit[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int ready = 0;
int high_backlight = 0;
int display_descriptor;

#pragma region pin definitions
int disp_array[128][64][2];
int numbers_array[5][8][10];
int pwm_pin = 18;
int fire_pin = 14;
int enc_clk_pin = 15;
int enc_data_pin = 17;
int enc_switch_pin = 27;
int enable_res_read_pin = 4;
int digit1_pin = 16;
int digit2_pin = 20;
int digit3_pin = 21;
int a_pin = 26;
int b_pin = 19;
int c_pin = 13;
int d_pin = 6;
int e_pin = 5;
int f_pin = 12;
int g_pin = 25;
int dot_pin = 24;
int ssd_pin[] = { 26, 19, 13, 6, 5, 12, 25, 24 };
int low_batt_pin = 22;
int backlight_enable_pin = 23;
#pragma end region


//display stuff
void init_display() {
	display_descriptor = wiringPiSPISetup(0, 1800000);
	unsigned char c = 0X30;
	wiringPiSPIDataRW(0, &c, 1);
	wiringPiSPIDataRW(0, &c, 1);
	c = 0x0C;
	wiringPiSPIDataRW(0, &c, 1);
	c = 0x34;
	wiringPiSPIDataRW(0, &c, 1);
	wiringPiSPIDataRW(0, &c, 1);
	c = 0x36;
	wiringPiSPIDataRW(0, &c, 1);
}

void draw_display() {
	for (int i = 0; i < 64; i++) {
		int line[16];
		for (int j = 0; j < 16; j++) line[i] = 0b00000000 | disp_array[j * 8 + 0][i][0] << 7 | disp_array[j * 8 + 1][i][0] << 6 | disp_array[j * 8 + 2][i][0] << 5 | disp_array[j * 8 + 3][i][0] << 4 | disp_array[j * 8 + 4][i][0] << 3 | disp_array[j * 8 + 5][i][0] << 2 | disp_array[j * 8 + 6][i][0] << 1 | disp_array[j * 8 + 7][i][0];

		unsigned char c = 0b11111000;
		wiringPiSPIDataRW(0, &c, 1);
		c = 0x80 + i % 32;
		wiringPiSPIDataRW(0, &c, 1);
		if (i >= 32) c = 0x88, wiringPiSPIDataRW(0, &c, 1);
		else c = 0x80, wiringPiSPIDataRW(0, &c, 1);

		c = 0b11111010;
		wiringPiSPIDataRW(0, &c, 1);
		for (int j = 0; j < 16; j++) c = line[j], wiringPiSPIDataRW(0, &c, 1);
	}
}

void write_resistance(float val) {
	int col = 78;
	int line = 11;

	//val = a.bc
	int a = (int)val;
	int b = (int)((val - a) * 10);
	int c = (int)(((val - a) * 10 - b) * 10);

	for (int i = 0; i < 8; i++)
		for (int j = 0; j < 5; j++) {
			disp_array[col + j][line + i][0] = numbers_array[j][i][a];
			disp_array[col + j + 11][line + i][0] = numbers_array[j][i][b];
			disp_array[col + j + 18][line + i][0] = numbers_array[j][i][c];
		}
}

void write_voltage(float val) {
	int col = 57;
	int line = 22;

	//val = a.bc
	int a = (int)val;
	int b = (int)((val - a) * 10);
	int c = (int)(((val - a) * 10 - b) * 10);

	for (int i = 0; i < 8; i++)
		for (int j = 0; j < 5; j++) {
			disp_array[col + j][line + i][0] = numbers_array[j][i][a];
			disp_array[col + j + 11][line + i][0] = numbers_array[j][i][b];
			disp_array[col + j + 18][line + i][0] = numbers_array[j][i][c];
		}
}

void write_current(float val) {
	int col = 57;
	int line = 33;

	//val = abc.d
	int a = (int)(val / 100.0);
	int b = (int)(val / 10.0 - a * 10);
	int c = (int)(val - a * 100 - b * 10);
	int d = (int)(val * 10 - a * 1000 - b * 100 - c * 10);

	for (int i = 0; i < 8; i++)
		for (int j = 0; j < 5; j++) {
			disp_array[col + j][line + i][0] = numbers_array[j][i][a];
			disp_array[col + j + 7][line + i][0] = numbers_array[j][i][b];
			disp_array[col + j + 14][line + i][0] = numbers_array[j][i][c];
			disp_array[col + j + 25][line + i][0] = numbers_array[j][i][d];
		}
}

void write_power(int val) {
	int col = 47;
	int line = 44;

	//val = abcd
	int a = val / 1000;
	int b = val / 100 - a * 10;
	int c = val / 10 - a * 100 - b * 10;
	int d = val - a * 1000 - b * 100 - c * 10;

	for (int i = 0; i < 8; i++)
		for (int j = 0; j < 5; j++) {
			disp_array[col + j][line + i][0] = numbers_array[j][i][a];
			disp_array[col + j + 7][line + i][0] = numbers_array[j][i][b];
			disp_array[col + j + 14][line + i][0] = numbers_array[j][i][c];
			disp_array[col + j + 21][line + i][0] = numbers_array[j][i][d];
		}
}

void write_temperature(int val) {
	int col = 47;
	int line = 44;

	//val = ab
	int a = val / 10;
	int b = val - a * 10;

	for (int i = 0; i < 8; i++)
		for (int j = 0; j < 5; j++) {
			disp_array[col + j][line + i][0] = numbers_array[j][i][a];
			disp_array[col + j + 7][line + i][0] = numbers_array[j][i][b];
		}
}

// interrupt functions
void interrupt_digit1() {
	for (int i = 0; i < 8; i++) digit[i] = wiringPi.digitalRead(ssd_pin[i]);
	ready = 1;
	system("/usr/local/bin/gpio edge 16 none");
}

void interrupt_digit2() {
	for (int i = 0; i < 8; i++) digit[i] = wiringPi.digitalRead(ssd_pin[i]);
	ready = 1;
	system("/usr/local/bin/gpio edge 20 none");
}

void interrupt_digit3() {
	for (int i = 0; i < 8; i++) digit[i] = wiringPi.digitalRead(ssd_pin[i]);
	ready = 1;
	system("/usr/local/bin/gpio edge 21 none");
}

//other stuff
int set_output(int currentPWM, int currentFeedback, int desiredFeedback) {
	int returnedPWM = currentPWM;
	if (currentFeedback > desiredFeedback and returnedPWM > 0)
		returnedPWM -= 1;
	if (currentFeedback < desiredFeedback and returnedPWM < 990)
		returnedPWM += 1;
	return returnedPWM;
}

int abdcefg_to_dec(int x) {
	switch (x) {
	case 63: return 0;
	case 6: return 1;
	case 91: return 2;
	case 79: return 3;
	case 102: return 4;
	case 109: return 5;
	case 125: return 6;
	case 7: return 7;
	case 127: return 8;
	case 103: return 9;
	case 111: return 9;
	default: return 0;
	}
	return 0;
}

void sleep_ms(int milliseconds) {
	struct timespec ts;
	ts.tv_sec = milliseconds / 1000;
	ts.tv_nsec = (milliseconds % 1000) * 1000000;
	nanosleep(&ts, NULL);
}

void read_display_matrix() {
	int fd = open("display.pixmap", O_RDONLY);
	for (int i = 0; i < 64; i++)
		for (int j = 0; j < 128; j++) {
			unsigned char c;
			read(fd, &c, 1);
			if (c == '8') c = '0';
			disp_array[j][i][0] = disp_array[j][i][1] = c - '0';
			read(fd, &c, 1);
		}
	close(fd);
}

void read_digits_matrix() {
	int fd = open("digits.pixmap", O_RDONLY);
	for (int i = 0; i < 10; i++)
		for (int j = 0; j < 8; j++)
			for (int k = 0; k < 5; k++) {
				unsigned char c;
				read(fd, &c, 1);
				numbers_array[k][j][i] = c - '0';
				read(fd, &c, 1);
			}
	close(fd);
}

long long current_timestamp() {
	struct timeval te;
	gettimeofday(&te, NULL); // get current time
	long long milliseconds = te.tv_sec * 1000LL + te.tv_usec / 1000; // caculate milliseconds
	return milliseconds;
}

int get_temp(int val) {
	float ntcRes = 5100 / ((4096.0 / val) - 1);
	float temp = ntcRes / 10000.0;
	temp = log(temp) / 3950 + 1.0 / 298.15;
	temp = 1.0 / temp;
	temp = temp - 273.15;
	return (int)temp;
}

void initialize_variables() {
	int fd = open("settings.conf", O_RDONLY);
	unsigned char reading[4] = { 0, 0, 0, 0 };
	read(fd, &reading, 4);
	resistance = reading[0] - '0' + (reading[2] - '0') / 10.0 + (reading[3] - '0') / 100.0;
	read(fd, &reading, 4);
	power = (reading[0] - '0') * 1000 + (reading[1] - '0') * 100 + (reading[2] - '0') * 10 + reading[3] - '0';
	voltage = sqrt(power * resistance);
	current = sqrt(power / resistance);
	int temp_read = read_voltage(0x68, 2, 12, 1, 0);
	temperature = get_temp(temp_read);

	enc_clk_last_state = wiringPi.digitalRead(enc_clk_pin);
}

void setup_gpio() {
	wiringPi.wiringPiSetupGpio();

	wiringPi.pinMode(backlight_enable_pin, 2); //sets backlight enable pin as output with pwm;
	wiringPi.pwmWrite(backlight_enable_pin, 190); //assume 0-255. high pwm, low brightness

	wiringPi.pinMode(pwm_pin, 2); //enables pcm clk pwm
	wiringPi.pwmSetClock(30); //sets freq to 320khz

	wiringPi.pinMode(fire_pin, 0); //sets firing pin as input
	wiringPi.pullUpDnControl(fire_pin, 2); //with pull up

	wiringPi.pinMode(enc_clk_pin, 0); //sets encoder clock pin as input
	wiringPi.pullUpDnControl(enc_clk_pin, 1); //with pull down

	wiringPi.pinMode(enc_data_pin, 0); //sets encoder data pin as input
	wiringPi.pullUpDnControl(enc_data_pin, 1); //with pull down

	wiringPi.pinMode(enc_switch_pin, 0); //sets encoder switch pin as input
	wiringPi.pullUpDnControl(enc_switch_pin, 1); //with pull down

	wiringPi.pinMode(enable_res_read_pin, 1); //sets read resistance pin as output

	wiringPi.pinMode(digit1_pin, 0); //digit 1 as input
	wiringPi.pullUpDnControl(digit1_pin, 2); //with pull up
	wiringPiISR(digit1_pin, INT_EDGE_FALLING, &interrupt_digit1);
	system("/usr/local/bin/gpio edge 16 none");

	wiringPi.pinMode(digit2_pin, 0); //digit 2 as input
	wiringPi.pullUpDnControl(digit2_pin, 2); //with pull up
	wiringPiISR(digit2_pin, INT_EDGE_FALLING, &interrupt_digit2);
	system("/usr/local/bin/gpio edge 20 none");

	wiringPi.pinMode(digit3_pin, 0); //digit 3 as input
	wiringPi.pullUpDnControl(digit3_pin, 2); //with pull up
	wiringPiISR(digit3_pin, INT_EDGE_FALLING, &interrupt_digit3);
	system("/usr/local/bin/gpio edge 21 none");

	wiringPi.pinMode(a_pin, 0); //a as input
	wiringPi.pullUpDnControl(a_pin, 1); //with pull down

	wiringPi.pinMode(b_pin, 0); //b as input
	wiringPi.pullUpDnControl(b_pin, 1); //with pull down

	wiringPi.pinMode(c_pin, 0); //c as input
	wiringPi.pullUpDnControl(c_pin, 1); //with pull down

	wiringPi.pinMode(d_pin, 0); //d as input
	wiringPi.pullUpDnControl(d_pin, 1); //with pull down

	wiringPi.pinMode(e_pin, 0); //e as input
	wiringPi.pullUpDnControl(e_pin, 1); //with pull down

	wiringPi.pinMode(f_pin, 0); //f as input
	wiringPi.pullUpDnControl(f_pin, 1); //with pull down

	wiringPi.pinMode(g_pin, 0); //g as input
	wiringPi.pullUpDnControl(g_pin, 1); //with pull down

	wiringPi.pinMode(dot_pin, 0); //dot as input
	wiringPi.pullUpDnControl(dot_pin, 1); //with pull down

	wiringPi.pinMode(low_batt_pin, 0); //low battery red as input
	wiringPi.pullUpDnControl(low_batt_pin, 1); //with pull down
}

void encoder_processing() {
	int clk_state = wiringPi.digitalRead(enc_clk_pin);
	int dt_state = wiringPi.digitalRead(enc_data_pin);
	if (clk_state != enc_clk_last_state) {
		high_backlight = 1;
		if (dt_state != clk_state) {
			//increase
			if (voltage < 9.0) power += 1;
		}
		else {
			//decrease
			if (voltage > 0.0) power -= 1;
		}
		//power is in w.between 0 and 5000
		voltage = sqrt(power * resistance);
		current = sqrt(power / resistance);
		computed_feedback = (int)(bias*voltage * 4096 / 9.0);
		write_voltage(voltage);
		write_current(current);
		write_power(power);
		draw_display();
		enc_clk_last_state = clk_state;
	}
}

void check_temp() {
	int temp_read = read_voltage(0x68, 2, 12, 1, 0);
	int tmp = get_temp(temp_read);
	if (tmp != temperature) {
		temperature = tmp;
		if (temperature > 70) high_temp = 1;
		if (temperature < 50) high_temp = 0;
		if (current_timestamp() - temp_write_time > 500) {
			write_temperature(temperature);
			draw_display();
			temp_write_time = current_timestamp();
		}
	}
}

void check_battery() {
	if (wiringPi.digitalRead(low_batt_pin)) low_batt_alarm = 1;
}

void compute_status() {
	if (high_temp == 1) {
		if (low_batt_alarm == 1)
			status = 3;
		else
			status = 2;
	}
	else {
		if (low_batt_alarm == 1)
			status = 1;
		else
			status = 0;
	}
}

void read_resistance() {
	if (wiringPi.digitalRead(enc_switch_pin) == 1) {
		high_backlight = 1;
		set_backlight();

		wiringPi.digitalWrite(enable_res_read_pin, 1);
		sleep_ms(2000);

		system("/usr/local/bin/gpio edge 16 falling");
		while (ready == 0);
		ready = 0;
		int ssd = digit[0] + digit[1] * 2 + digit[2] * 4 + digit[3] * 8 + digit[4] * 16 + digit[5] * 32 + digit[6] * 64;
		resistance = abdcefg_to_dec(ssd);

		system("/usr/local/bin/gpio edge 20 falling");
		while (ready == 0);
		ready = 0;
		ssd = digit[0] + digit[1] * 2 + digit[2] * 4 + digit[3] * 8 + digit[4] * 16 + digit[5] * 32 + digit[6] * 64;
		resistance += abdcefg_to_dec(ssd) * 0.1;

		system("/usr/local/bin/gpio edge 21 falling");
		while (ready == 0);
		ready = 0;
		ssd = digit[0] + digit[1] * 2 + digit[2] * 4 + digit[3] * 8 + digit[4] * 16 + digit[5] * 32 + digit[6] * 64;
		resistance += abdcefg_to_dec(ssd) * 0.01;

		write_resistance(resistance);
		draw_display();

		wiringPi.digitalWrite(enable_res_read_pin, 0);
	}
}

void try_fire() {
	long long firing_length = current_timestamp();
	while ((wiringPi.digitalRead(fire_pin) != 1) and (status == 0) and (current_timestamp() - firing_length < 7000)) {
		high_backlight = 1;
		set_backlight();
		wiringPi.pwmWrite(pwm_pin, output_pwm);
		output_pwm = set_output(output_pwm, read_voltage(0x68, 1, 12, 1, 0), computed_feedback);
	}
}

void set_backlight() {
	if (high_backlight == 1) {
		wiringPi.pwmWrite(backlight_enable_pin, 60);
		backlight_time = current_timestamp();
		high_backlight = 0;
	}
	else if (current_timestamp() - backlight_time > 5000) {
		wiringPi.pwmWrite(backlight_enable_pin, 190);
	}
}

int main() {
	setup_gpio();

	read_display_matrix();
	read_digits_matrix();
	init_display();

	initialize_variables();
	write_resistance(resistance);
	write_voltage(voltage);
	write_current(current);
	write_power(power);
	write_temperature(temperature);

	draw_display();

	//main while loop
	while (true) {

		check_temp();
		check_battery();
		compute_status();
		read_resistance();
		encoder_processing();
		set_backlight();
		try_fire();

	}
}