//compile with gcc ecig.cpp -lwiringPi -lm -o ecig
//run with sudo ./ecig

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "wiringPi/wiringPi/wiringPi.h"
#include "wiringPi/wiringPi/wiringPiSPI.h"
#include <time.h>
#include <errno.h>
#include <linux/i2c-dev.h>

//for ADC lib
static int i2cbus;
const char *fileName = "/dev/i2c-1"; // change to /dev/i2c-0 if you are using a revision 0002 or 0003 model B
unsigned char writebuffer[10] = { 0 };
unsigned char readbuffer[10] = { 0 };
static char sbit = 0;


static void open_i2c_bus() {
	if ((i2cbus = open(fileName, O_RDWR)) < 0) {
		exit(1);
	}
	printf("Opened i2c bus\n");
}

static void close_i2c_bus() {
	close(i2cbus);
}

static void read_byte_array(char address, char reg, char length) {

	if (ioctl(i2cbus, I2C_SLAVE, address) < 0) {
		printf("Failed to write to i2c port for read\n");
		exit(1);
	}

	writebuffer[0] = reg;

	if ((write(i2cbus, writebuffer, 1)) != 1) {
		printf("Failed to write to i2c device for read\n");
		exit(1);
	}

	read(i2cbus, readbuffer, 4);
}

static char update_byte(char byte, char bit, char value) {
	/*
	internal method for setting the value of a single bit within a byte
	*/
	if (value == 0) {
		return (byte &= ~(1 << bit));

	}
	else {
		return (byte |= 1 << bit);
	}

}

static char set_pga(char config, char gain) {
	/*
	internal method for Programmable Gain Amplifier gain selection
	*/
	switch (gain) {
	case 1:
		config = update_byte(config, 0, 0);
		config = update_byte(config, 1, 0);
		break;
	case 2:
		config = update_byte(config, 0, 1);
		config = update_byte(config, 1, 0);
		break;
	case 4:
		config = update_byte(config, 0, 0);
		config = update_byte(config, 1, 1);
		break;
	case 8:
		config = update_byte(config, 0, 1);
		config = update_byte(config, 1, 1);
		break;
	default:
		break;
	}
	return (config);
}

static char set_bit_rate(char config, char rate) {
	/*
	internal method for bit rate selection
	*/
	switch (rate) {
	case 12:
		config = update_byte(config, 2, 0);
		config = update_byte(config, 3, 0);
		break;
	case 14:
		config = update_byte(config, 2, 1);
		config = update_byte(config, 3, 0);
		break;
	case 16:
		config = update_byte(config, 2, 0);
		config = update_byte(config, 3, 1);

		break;
	case 18:
		config = update_byte(config, 2, 1);
		config = update_byte(config, 3, 1);

		break;
	default:
		break;
	}
	return (config);
}

static char set_conversion_mode(char config, char mode) {
	/*
	internal method for setting the conversion mode
	*/
	if (mode == 1) {
		config = update_byte(config, 4, 1);
	}
	else {
		config = update_byte(config, 4, 0);
	}

	return (config);
}

static char set_channel(char config, char channel) {
	/*
	internal method for setting the channel
	*/
	switch (channel) {
	case 1:
		config = update_byte(config, 5, 0);
		config = update_byte(config, 6, 0);
		break;
	case 2:
		config = update_byte(config, 5, 1);
		config = update_byte(config, 6, 0);
		break;
	case 3:
		config = update_byte(config, 5, 0);
		config = update_byte(config, 6, 1);
		break;
	case 4:
		config = update_byte(config, 5, 1);
		config = update_byte(config, 6, 1);
		break;
	}

	return (config);
}

/**
* Reads the raw value from the selected ADC channel
* @param address - I2C address for the target device e.g. 0x68
* @param channel - 1 to 4
* @param bitrate - 12, 14, 16 or 18
* @param pga - 1, 2, 4 or 8
* @param conversionmode - 0 = one shot conversion, 1 = continuous conversion
* @returns - raw long value from ADC buffer
*/
int read_raw(char address, char channel, int bitrate, int pga,
	char conversionmode) {
	// variables for storing the raw bytes from the ADC
	char h = 0;
	char l = 0;
	char m = 0;
	char s = 0;
	char config = 0x9C;
	long t = 0;
	sbit = 0;

	// set the config based on the provided parameters
	config = set_channel(config, channel);
	config = set_conversion_mode(config, conversionmode);
	config = set_bit_rate(config, bitrate);
	config = set_pga(config, pga);

	// keep reading the ADC data until the conversion result is ready
	int timeout = 1000; // number of reads before a timeout occurs
	int x = 0;

	open_i2c_bus();

	do {
		if (bitrate == 18) {
			read_byte_array(address, config, 3);
			h = readbuffer[0];
			m = readbuffer[1];
			l = readbuffer[2];
			s = readbuffer[3];
		}
		else {
			read_byte_array(address, config, 2);
			h = readbuffer[0];
			m = readbuffer[1];
			s = readbuffer[2];
		}

		// check bit 7 of s to see if the conversion result is ready
		if (!(s & (1 << 7))) {
			break;
		}

		if (x > timeout) {
			// timeout occurred
			return (0);
		}

		x++;
	} while (1);

	close_i2c_bus();

	// extract the returned bytes and combine in the correct order
	switch (bitrate) {
	case 18:
		t = ((h & 3) << 16) | (m << 8) | l;
		if ((t >> 17) & 1) {
			sbit = 1;
			t &= ~(1 << 17);
		}
		break;
	case 16:
		t = (h << 8) | m;
		if ((t >> 15) & 1) {
			sbit = 1;
			t &= ~(1 << 15);
		}
		break;
	case 14:
		t = ((h & 63) << 8) | m;
		if ((t >> 13) & 1) {
			sbit = 1;
			t &= ~(1 << 13);
		}
		break;
	case 12:
		t = ((h & 15) << 8) | m;
		if ((t >> 11) & 1) {
			sbit = 1;
			t &= ~(1 << 11);
		}
		break;
	default:
		break;
	}

	return (t);
}

/**
* Returns the voltage from the selected ADC channel
* @param address - I2C address for the target device e.g. 0x68
* @param channel - 1 to 4
* @param bitrate - 12, 14, 16 or 18
* @param pga - 1, 2, 4 or 8
* @param conversionmode - 0 = one shot conversion, 1 = continuous conversion
* @returns - double voltage value from ADC
*/
double read_voltage(char address, char channel, int bitrate, int pga,
	char conversionmode) {
	int raw = read_raw(address, channel, bitrate, pga, conversionmode); // get the raw value

																		// calculate the gain based on the pga value
	double gain = (double)pga / 2;

	// set the lsb value based on the bitrate
	double lsb = 0;

	switch (bitrate) {
	case 12:
		lsb = 0.0005;
		break;
	case 14:
		lsb = 0.000125;
		break;
	case 16:
		lsb = 0.00003125;
		break;
	case 18:
		lsb = 0.0000078125;
		break;
	default:
		return (9999);
		break;
	}

	if (sbit == 1) // if the signbit is 1 the value is negative and most likely noise so it can be ignored.
	{
		return (0);
	}
	else {
		double voltage = ((double)raw * (lsb / gain)) * 2.471; // calculate the voltage and return it
		return (voltage);
	}
}

//


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
	printf("Opened spi bus\n");
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
	for (int i = 0; i < 8; i++) digit[i] = digitalRead(ssd_pin[i]);
	ready = 1;
	system("/usr/local/bin/gpio edge 16 none");
}

void interrupt_digit2() {
	for (int i = 0; i < 8; i++) digit[i] = digitalRead(ssd_pin[i]);
	ready = 1;
	system("/usr/local/bin/gpio edge 20 none");
}

void interrupt_digit3() {
	for (int i = 0; i < 8; i++) digit[i] = digitalRead(ssd_pin[i]);
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
	struct timespec te;
	clock_gettime(CLOCK_REALTIME, &te); // get current time
	long long milliseconds = te.tv_sec * 1000LL + te.tv_nsec / 1000000LL; // caculate milliseconds
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

	enc_clk_last_state = digitalRead(enc_clk_pin);
}

void setup_gpio() {
	wiringPiSetupGpio();

	pinMode(backlight_enable_pin, 2); //sets backlight enable pin as output with pwm;
	pwmWrite(backlight_enable_pin, 190); //assume 0-255. high pwm, low brightness

	pinMode(pwm_pin, 2); //enables pcm clk pwm
	pwmSetClock(30); //sets freq to 320khz

	pinMode(fire_pin, 0); //sets firing pin as input
	pullUpDnControl(fire_pin, 2); //with pull up

	pinMode(enc_clk_pin, 0); //sets encoder clock pin as input
	pullUpDnControl(enc_clk_pin, 1); //with pull down

	pinMode(enc_data_pin, 0); //sets encoder data pin as input
	pullUpDnControl(enc_data_pin, 1); //with pull down

	pinMode(enc_switch_pin, 0); //sets encoder switch pin as input
	pullUpDnControl(enc_switch_pin, 1); //with pull down

	pinMode(enable_res_read_pin, 1); //sets read resistance pin as output

	pinMode(digit1_pin, 0); //digit 1 as input
	pullUpDnControl(digit1_pin, 2); //with pull up
	wiringPiISR(digit1_pin, INT_EDGE_FALLING, &interrupt_digit1);
	system("/usr/local/bin/gpio edge 16 none");

	pinMode(digit2_pin, 0); //digit 2 as input
	pullUpDnControl(digit2_pin, 2); //with pull up
	wiringPiISR(digit2_pin, INT_EDGE_FALLING, &interrupt_digit2);
	system("/usr/local/bin/gpio edge 20 none");

	pinMode(digit3_pin, 0); //digit 3 as input
	pullUpDnControl(digit3_pin, 2); //with pull up
	wiringPiISR(digit3_pin, INT_EDGE_FALLING, &interrupt_digit3);
	system("/usr/local/bin/gpio edge 21 none");

	pinMode(a_pin, 0); //a as input
	pullUpDnControl(a_pin, 1); //with pull down

	pinMode(b_pin, 0); //b as input
	pullUpDnControl(b_pin, 1); //with pull down

	pinMode(c_pin, 0); //c as input
	pullUpDnControl(c_pin, 1); //with pull down

	pinMode(d_pin, 0); //d as input
	pullUpDnControl(d_pin, 1); //with pull down

	pinMode(e_pin, 0); //e as input
	pullUpDnControl(e_pin, 1); //with pull down

	pinMode(f_pin, 0); //f as input
	pullUpDnControl(f_pin, 1); //with pull down

	pinMode(g_pin, 0); //g as input
	pullUpDnControl(g_pin, 1); //with pull down

	pinMode(dot_pin, 0); //dot as input
	pullUpDnControl(dot_pin, 1); //with pull down

	pinMode(low_batt_pin, 0); //low battery red as input
	pullUpDnControl(low_batt_pin, 1); //with pull down
}

void encoder_processing() {
	int clk_state = digitalRead(enc_clk_pin);
	int dt_state = digitalRead(enc_data_pin);
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
	if (digitalRead(low_batt_pin)) low_batt_alarm = 1;
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

void set_backlight() {
	if (high_backlight == 1) {
		pwmWrite(backlight_enable_pin, 60);
		backlight_time = current_timestamp();
		high_backlight = 0;
	}
	else if (current_timestamp() - backlight_time > 5000) {
		pwmWrite(backlight_enable_pin, 190);
	}
}

void read_resistance() {
	if (digitalRead(enc_switch_pin) == 1) {
		high_backlight = 1;
		set_backlight();

		digitalWrite(enable_res_read_pin, 1);
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

		digitalWrite(enable_res_read_pin, 0);
	}
}

void try_fire() {
	long long firing_length = current_timestamp();
	while ((digitalRead(fire_pin) != 1) and (status == 0) and (current_timestamp() - firing_length < 7000)) {
		high_backlight = 1;
		set_backlight();
		pwmWrite(pwm_pin, output_pwm);
		output_pwm = set_output(output_pwm, read_voltage(0x68, 1, 12, 1, 0), computed_feedback);
	}
}

int main() {
	printf("hello\n");
	setup_gpio();
	printf("gpio set up\n");

	read_display_matrix();
	printf("display matrix read\n");
	read_digits_matrix();
	printf("digit matrices read\n");
	init_display();
	printf("display initialized\n");

	initialize_variables();
	printf("variables initialized\n");
	write_resistance(resistance);
	write_voltage(voltage);
	write_current(current);
	write_power(power);
	write_temperature(temperature);
	printf("values written in display buffer\n");

	draw_display();
	printf("first display frame printed\n");

	//main while loop
	printf("entering while loop\n");
	while (true) {

		check_temp();
		printf("temp checked\n");
		check_battery();
		printf("battery checked\n");
		compute_status();
		printf("status computed\n");
		read_resistance();
		printf("resistance read\n");
		encoder_processing();
		printf("encoder processed\n");
		set_backlight();
		printf("backlight set\n");
		try_fire();
		printf("tried to fire\n");

	}
}