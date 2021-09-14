#include"sra.c"
int mpos=0;
unsigned int opt = 265;
float kp = 15;  //Proportional Gain;
float control;
float prev_error;
float act_error;
float kd = 3.1;
int flcount = 0, l = 0, lcount = 0, straight = 0 , lcountPrev = 0 , flag1 = 0, irc = 0;
int lcount_forStraight_prev = 0 , temp = 0 , temp_forStraight = 0 ;
char turn = 's';
int dir = 2, Coordinates_changed_by = 0 ;
int x_coordinate = 5, y_coordinate = 5 ;
// int dir_array[9][9][4];
int map_array[9][9];
int mx, my;
int new_array[9][2] = {{5,5},{5,6},{5,7},{6,7},{7,7},{7,8},{6,8},{5,8},{5,9}};
void line_track(void)	
{
	//check_sensors();
	bot_forward();
	switch(sensorbyte)
	{ 
		case 0b0110:mpos = 0; break;//sensor numbering starts from right

		case 0b0100:mpos = 1; break;

		case 0b0010:mpos = -1; break;

		case 0b1100:mpos = 3; break;
		
		case 0b0011:mpos = -3; 
		//lcd_clear();
		//lcd_write_string("0011");
		if(flag1==1)
		{
			mpos = 0;
		}
		break;

		case 0b1110:mpos = 4; break;
		
		case 0b0111:mpos = -4; 
		//lcd_clear();
		//lcd_write_string("0111");
		if(flag1==1)
		{
			mpos = 0;
		}
		break;
		
		
		case 0b1000:mpos = 6; break;

		case 0b0001:mpos = -6; break;
		
		// case 0b0000:
		// if(flag1==0)
		// {
		// 	lcd_write_string("uturn");
		// 	Uturn();
		// 	bot_stop();
		// 	delay_millisec(100);
		// }
		break;
		default: break;
	}
	
	act_error = prev_error - mpos;
	control = (mpos * kp) - (kd*(act_error));
	prev_error = mpos ;
	int motor1_value = opt - control;//right motor is m1
	int motor2_value = opt + control;
	set_pwm1a(motor1_value);
	set_pwm1b(motor2_value);
	
	//delay_microsec(10);
}

void init_devices(void)
{ 
	port_init();
	adc_init();
	bot_motion_init();
	lcd_init(underline);
	switch_init();
	pwm1_init();
}

void Uturn()
{	
	turn = 'u';
	
	while(bit_is_set(PINA,5))
	{
		set_pwm1a(240);
		set_pwm1b(290);
		bot_spot_left();
		//left_count();
	}
	while((bit_is_set(PINA,6) && bit_is_set(PIND, 6)))
	{
		set_pwm1a(185);
		set_pwm1b(215);
		bot_spot_left();
		//left_count();
	}
	set_pwm1a(399);
	set_pwm1b(399);
	bot_brake();
	//delay_sec(2);
}
void turn_right()
{	
	turn = 'r';
	while(bit_is_set(PINA,7))
	{
		set_pwm1a(255);
		set_pwm1b(255);
		bot_spot_right();
		left_count();
	}
	while((bit_is_set(PINA,6) && bit_is_set(PIND, 6)))
	{
		set_pwm1a(205);
		set_pwm1b(205);
		bot_spot_right();
		left_count();
	}
	while(bit_is_clear(PINA,4) || bit_is_clear(PIND,7))
	{
		line_track_new();
	}
	
	set_pwm1a(399);
	set_pwm1b(399);
	bot_brake();
	delay_sec(1);
	//delay_millisec(100);
}

void turn_left()
{	
	turn = 'l';
	while(bit_is_set(PINA,5))
	{
		set_pwm1a(265);
		set_pwm1b(265);
		bot_spot_left();
	}
	while(bit_is_set(PINA,6) && bit_is_set(PIND,6))
	{
		set_pwm1a(205);
		set_pwm1b(205);
		bot_spot_left();
	}
	while(bit_is_clear(PINA,4) || bit_is_clear(PIND,7))
	{
		line_track_new();
	}
	set_pwm1a(399);
	set_pwm1b(399);
	bot_brake();
	delay_sec(1);
}

void left_count()
{
	if(bit_is_clear(PINA,4))
	{
		if(l==0)
		{
			lcount++;
			l = 1;
		}
	}
	else
	{
		l = 0;
	}
	
}


void line_track_new()
{
	check_sensors();
	bot_forward();
	if(bit_is_set(PINA,6) && bit_is_clear(PIND,6))
	{
		set_pwm1a(230);
		set_pwm1b(290);

	}
	else if(bit_is_set(PIND,6) && bit_is_clear(PINA,6))
	{
		set_pwm1a(290);
		set_pwm1b(230);
	}
	else if(bit_is_set(PIND,6) && bit_is_set(PINA,6))
	{
		line_track();
	}
	else if(bit_is_clear(PIND,6) && bit_is_clear(PINA,6))
	{
		set_pwm1a(260);
		set_pwm1b(260);
	}
		delay_microsec(10);
		irc++;
}

void sense_of_directon(void)
{
	if(turn == 'r')	
	{
		switch(dir)
		{
			case +1: dir = -2; break;
			case -1: dir = +2; break;
			case +2: dir = +1; break;
			case -2: dir = -1; break;	
		}
	}
	else if(turn == 'l')	
	{
		switch(dir)
		{
			case +1: dir = +2; break;
			case -1: dir = -2; break;			
			case +2: dir = -1; break;
			case -2: dir = +1; break;	
		}
	}
	else if(turn == 'u')	
	{
		switch(dir)
		{
			case +1: dir = -1; break;
			case -1: dir = +1; break;		
			case +2: dir = -2; break;
			case -2: dir = +2; break;	
		}
	}
	
}


void go_to_coordinate(int next_x , int next_y )
{
	if (next_x-x_coordinate>0)
	{	
		checkAndCorrectDirection(1);
		while(bit_is_set(PINA,4) && bit_is_set(PIND,7))
		{
		line_track_new();
		}
		set_pwm1b(399);
		set_pwm1a(399);
		bot_brake();
		delay_millisec(500);
	}
	if(next_x-x_coordinate<0)
	{
		checkAndCorrectDirection(-1);
		while(bit_is_set(PINA,4) && bit_is_set(PIND,7))
		{
		line_track_new();
		}
		bot_brake();
	}

	if (next_y-y_coordinate>0)
	{	
		checkAndCorrectDirection(2);
		while(bit_is_set(PINA,4) && bit_is_set(PIND,7))
		{
		line_track_new();
		}
		bot_brake();
	}
	if(next_y-y_coordinate<0)
	{
		checkAndCorrectDirection(-2);
		while(bit_is_set(PINA,4) && bit_is_set(PIND,7))
		{
		line_track_new();
		}
		bot_brake();
	}	
	x_coordinate = next_x;
	y_coordinate = next_y;
	lcd_clear();
	lcd_write_int_xy(0,1,x_coordinate,2);
	lcd_write_int_xy(5,1,y_coordinate,2);
	lcd_write_int_xy(3,0,irc,5);
	bot_brake();
	delay_millisec(500);
}


void checkAndCorrectDirection(int req_dir)
{
	if(req_dir == 1)
	{
		switch(dir)
		{	
			
			case +2: turn_right();sense_of_directon();break;

			case -1:turn_left();sense_of_directon();break;

		}
	}

	if(req_dir == 2)
	{
		switch(dir)
		{	lcd_clear();
			lcd_write_int_xy(0,0,dir,3);
			bot_stop();
			delay_sec(1);
			case +1: turn_left();sense_of_directon();break;
			case -1:turn_right();sense_of_directon();break;

		}
	}

	if(req_dir == -1)
	{
		switch(dir)
		{	
			case +2: turn_left();sense_of_directon();break;
		
			case -2:turn_right();sense_of_directon();break;

		}
	}


	if(req_dir == -2)
	{
		switch(dir)
		{	
			case +1:turn_right();sense_of_directon();break; 
				
			case -1: turn_left();sense_of_directon();break;
					
			

		}
	}
	bot_brake();
	delay_sec(1);

}


int main(void)
{
	init_devices();
	lcd_clear();
	lcd_write_string("LCD Working");
	int i, j, k;
	//array initializing to zero
	//for(i=0 ; i<9 ; i++)
	//{
		//for(j=0 ; j<9 ; j++)
		//{
			//for(k=0 ; k<4 ; k++)
			//{
				//dir_array[i][j][k] = 0;
			//}
		//}
	//}
	lcd_clear();
	lcd_write_string("Press Any Key");
	//update_array(1,0,0,0);
	while(1)
	{	
		if(pressed_switch2() || pressed_switch0() || pressed_switch1() || pressed_switch3())
		{
			lcd_clear();
			lcd_write_string("pressed");
			while(1)
			{
				// line_track_new();	
				LED&=0b11110000;
				LED|=sensorbyte;
				// right_junc_check();
				// left_junc_check();
				// frSensorCheck();
				// irc++;
			lcd_clear();
			lcd_write_string("while");
			delay_sec(2);
			
				for (int p = 1; p <= 8; p++)
				{
					
						lcd_clear();
						lcd_write_string("for");
						if(!((p==1 ) || (p==6 )))
						{	
							lcd_clear();
							lcd_write_int_xy(0,0,p,2);
							go_to_coordinate(new_array[p][0],new_array[p][1]);
							LED&=0b11110000;
							LED|=sensorbyte;

						}
				}

			}
		}
	}
	return 0;
}