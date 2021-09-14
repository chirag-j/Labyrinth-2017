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
int dir_array[9][9][4];
int mx, my;
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
		set_pwm1a(200);
		set_pwm1b(200);
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
		set_pwm1a(200);
		set_pwm1b(200);
		bot_spot_left();
	}
	while(bit_is_clear(PINA,4) || bit_is_clear(PIND,7))
	{
		line_track_new();
	}
	set_pwm1a(399);
	set_pwm1b(399);
	bot_brake();
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

void special_case_check()         ////called inside left_junc_check
{
	if(bit_is_clear(PINA,5) || flag1 == 1)
	{
		flag1 = 1;
		
		if(bit_is_clear(PINA,4) && (bit_is_clear(PIND,6) || bit_is_clear(PINA,6)))
		{
			while(bit_is_clear(PINA,4) && bit_is_set(PIND, 7)) 
			{
				set_pwm1a(240);
				set_pwm1b(240);
				bot_forward();
				flag1 = 0;
				if(bit_is_clear(PIND, 7))
				{
					bot_brake();
				}
			}
		}
	}
}
void right_junc_check()
{	
	
	if(bit_is_clear(PINA,7))
	{
		bot_brake();
		// lcd_clear();
		// lcd_write_int_xy(0,0,irc,5);
		// delay_sec(1);
		lcd_clear();
		if(irc>2000)
		{
			// lcd_write_string("Sixty");
			
			Coordinates_changed_by = 2;
		}
		else
		{
			// lcd_write_string("Thirty");
			Coordinates_changed_by = 1;
		}
		
		//delay_sec(2);
		update_coordinates();
		lcountPrev = lcount;
		while(bit_is_set(PIND, 7))
		{
			line_track_new();
			left_count();
		}
		//End Condition
		if(bit_is_clear(PINA,6) && bit_is_clear(PIND,6) && bit_is_clear(PINA,5) && bit_is_clear(PINA,7))
		{
			lcd_clear();
			bot_brake();
			fill_missed_array();
			coordinates_tobe_reached();
			flick();
			lcd_clear();
			lcd_write_string_xy(0,0,"mx: ");
			lcd_write_int_xy(4,0,mx,2);
			lcd_write_string_xy(07,0,"my: ");
			lcd_write_int_xy(11,0,my,2);
			lcd_write_string_xy(0,1,"Press a Key-->");
			while(1)
			{
				if(pressed_switch2() || pressed_switch0() || pressed_switch1() || pressed_switch3())
				{
					lcd_clear();
					//lcd_write_string("");
					delay_sec(1);
					break;
				}
			}
			
			lcd_write_string("Press Any Key");
			int i=5, j=5;
			while(1)
			{
				if(pressed_switch3())
				{
					lcd_clear();
					lcd_write_string("D3 Pressed");
					delay_sec(1);
					lcd_clear();
					j-=1;
					lcd_write_string("[");
					lcd_write_int_xy(2,0,dir_array[i][j][0],1);
					lcd_write_int_xy(4,0,dir_array[i][j][1],1);
					lcd_write_int_xy(6,0,dir_array[i][j][2],1);
					lcd_write_int_xy(8,0,dir_array[i][j][3],1);
					lcd_write_string_xy(10,0,"]");
					lcd_write_int_xy(0,1,i,1);
					lcd_write_int_xy(2,1,j,1);
				}
				if(pressed_switch2())
				{
					lcd_clear();
					lcd_write_string("D2 Pressed");
					delay_sec(1);
					lcd_clear();
					i-=1;
					lcd_write_string("[");
					lcd_write_int_xy(2,0,dir_array[i][j][0],1);
					lcd_write_int_xy(4,0,dir_array[i][j][1],1);
					lcd_write_int_xy(6,0,dir_array[i][j][2],1);
					lcd_write_int_xy(8,0,dir_array[i][j][3],1);
					lcd_write_string_xy(10,0,"]");
					lcd_write_int_xy(0,1,i,1);
					lcd_write_int_xy(2,1,j,1);
				}
				if(pressed_switch1())
				{
					lcd_clear();
					lcd_write_string("D1 Pressed");
					delay_sec(1);
					lcd_clear();
					i+=1;
					lcd_write_string("[");
					lcd_write_int_xy(2,0,dir_array[i][j][0],1);
					lcd_write_int_xy(4,0,dir_array[i][j][1],1);
					lcd_write_int_xy(6,0,dir_array[i][j][2],1);
					lcd_write_int_xy(8,0,dir_array[i][j][3],1);
					lcd_write_string_xy(10,0,"]");
					lcd_write_int_xy(0,1,i,1);
					lcd_write_int_xy(2,1,j,1);
				}
				if(pressed_switch0())
				{
					lcd_clear();
					lcd_write_string("D0 Pressed");
					delay_sec(1);
					lcd_clear();
					j+=1;
					lcd_write_string("[");
					lcd_write_int_xy(2,0,dir_array[i][j][0],1);
					lcd_write_int_xy(4,0,dir_array[i][j][1],1);
					lcd_write_int_xy(6,0,dir_array[i][j][2],1);
					lcd_write_int_xy(8,0,dir_array[i][j][3],1);
					lcd_write_string_xy(10,0,"]");
					lcd_write_int_xy(0,1,i,1);
					lcd_write_int_xy(2,1,j,1);
				}

			}
			
		}
		if(bit_is_clear(PINA,6) || bit_is_clear(PIND,6))
		{
			straight = 1;
		}
		else
		{
			straight = 0;
		}
		
		
		turn_right();						////actual turn
		
		l = 0;
		bot_brake();
		//delay_millisec(100);
		flag1 = 0;
		irc = 0;
		temp = lcount - lcountPrev;
		//lcd_clear();
		//lcd_write_string("lcount:");
		//lcd_write_int_xy(8,0,temp,2);
		//lcd_write_string_xy(0,1,"straight:");
		//lcd_write_int_xy(10,1,straight,2);
		// bot_stop();
		//delay_millisec(2000);
		// lcd_clear();
		
		//delay_sec(2);
		//lcd_clear();
		if(straight==1 && temp==1)
		{
			// lcd_write_string("|- Detected");
			update_array(1,1,1,0);
			lcd_write_string_xy(0,1,"1");
			lcd_write_string_xy(2,1,"1");
			lcd_write_string_xy(4,1,"1");
			lcd_write_string_xy(6,1,"0");
		}
		else if(straight==0 && temp==1)
		{
			// lcd_write_string("T Detected");
			update_array(0,1,1,1);
			lcd_write_string_xy(0,1,"0");
			lcd_write_string_xy(2,1,"1");
			lcd_write_string_xy(4,1,"1");
			lcd_write_string_xy(6,1,"1");
		}
		else if(straight==0 && temp==0)
		{
			// lcd_write_string("L Detected");
			update_array(0,1,1,0);
			lcd_write_string_xy(0,1,"0");
			lcd_write_string_xy(2,1,"1");
			lcd_write_string_xy(4,1,"1");
			lcd_write_string_xy(6,1,"0");
		}
		else if(straight==1 && temp==2)
		{
			// lcd_write_string("+ Detected");
			update_array(1,1,1,1);
			lcd_write_string_xy(0,1,"1");
			lcd_write_string_xy(2,1,"1");
			lcd_write_string_xy(4,1,"1");
			lcd_write_string_xy(6,1,"1");
		}
		//delay_millisec(2000);
		temp = 0;
		sense_of_directon();
		// lcd_clear();
		// lcd_write_int_xy(0,0,dir,3);
		//delay_sec(2);
		
	}
}
void frSensorCheck()
{
	if(bit_is_set(PINA,6) && bit_is_set(PIND,6))
	{
		////just confirming if its really a dead end or the bot has simply just wobbled
		if(sensorbyte==0b0011 || sensorbyte==0b1100 || sensorbyte==0b0100 || sensorbyte==0b0010 || sensorbyte==0b0110)
		{
			if(flag1==0)
			{
				//lcd_clear();
				// lcd_write_string("Uturn Detected");
				if(irc>2000)
				{
					// lcd_write_string("Sixty");
					if(Coordinates_changed_by ==0)
					{
						Coordinates_changed_by = 2;
					}
				}
				else
				{
					// lcd_write_string("Thirty");
					if(Coordinates_changed_by ==0)
					{
						Coordinates_changed_by = 1;
					}
				}
				//update_coordinates();
				//bot_brake();
				//delay_sec(2);
			}
		}
		else
		{
			lcd_clear();
			lcd_write_string("ERROR Fr Snsr");
		}
	}
	
	if(sensorbyte==0b0000 && bit_is_set(PINA,4) && bit_is_set(PINA,5) && bit_is_set(PINA,6) && bit_is_set(PINA,7) && bit_is_set(PIND, 6) && bit_is_set(PIND, 7) && flag1==0)
	{
		//lcd_clear();
		//lcd_write_string("Uturn");
		bot_brake();
		update_coordinates();
		update_array(0,0,1,0);
		//delay_sec(2);

		Uturn();
		//bot_brake();
		sense_of_directon();
		// lcd_clear();
		// lcd_write_int_xy(0,0,dir,3);
		// delay_sec(2);
		flag1 = 0;
		irc = 0;
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
}
void left_junc_check()                                 
{	

	if(bit_is_clear(PINA,5))
	{
		flag1 = 1;
		//bot_brake();
		// lcd_clear();
		// lcd_write_int_xy(0,0,irc,5);
		// delay_sec(1);
		if(irc>2000)
		{	
			lcd_clear();
			// lcd_write_string("sixty");
			Coordinates_changed_by = 2;
		}
		else
		{
			lcd_clear();
			// lcd_write_string("Thirty");
			Coordinates_changed_by = 1;
		}				
	}
	if((bit_is_clear(PINA,6) || bit_is_clear(PIND,6)) && bit_is_clear(PINA,4))
	{
		bot_brake();
		update_coordinates();
		//delay_sec(2);
		//lcd_clear();
		update_array(1,0,1,1);			
		// lcd_write_string("-| detected");
		//delay_millisec(500);
		irc = 0;
	}
	
	else if(bit_is_clear(PINA,4) && bit_is_set(PINA,6) && bit_is_set(PIND,6))
	{
		// lcd_clear();
		// bot_brake();
		// lcd_write_string("L detected");
		update_coordinates();
		//delay_sec(2);
		irc = 0;
		turn_left();						

		update_array(0,0,1,1);
		bot_brake();
		sense_of_directon();
		// lcd_clear();
		// lcd_write_int_xy(0,0,dir,3);
		// delay_millisec(2000);
		flag1 = 0;
	}

	special_case_check();       ///////////
	
}


void update_coordinates(void)
{	
	int value = Coordinates_changed_by;
	
	if(value==2)		//to store the middle point in 60cm shift i.e to make it continous
	{
		switch(dir)
		{
			case +1 :	x_coordinate+=1;						
						update_array(1,0,1,0);
						x_coordinate+=1;
						break;

			case -1 :	x_coordinate-=1;						
						update_array(1,0,1,0);
						x_coordinate-=1;
						break;
					
			case +2 :	y_coordinate+=1;						
						update_array(1,0,1,0);
						y_coordinate+=1;
						
						break;

			case -2 :	y_coordinate-=1;						
						update_array(1,0,1,0);	
						y_coordinate-=1;
						break;	
		}
	}
	else
	{
		switch(dir)
		{
			case +1 : x_coordinate+=value; break;
			case -1 : x_coordinate-=value; break;
			case +2 : y_coordinate+=value; break;
			case -2 : y_coordinate-=value; break;	
		}
	}
	lcd_clear();
	lcd_write_int_xy(4,0,x_coordinate,2);
	lcd_write_int_xy(8,0,y_coordinate,2);
	Coordinates_changed_by = 0;
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

void update_array(int y,int x,int ny,int nx)
{
	////checking if the path is already traversed before i.e(if all 4 values is zero)
	if(dir_array[x_coordinate][y_coordinate][0]==0 && dir_array[x_coordinate][y_coordinate][1]==0 && dir_array[x_coordinate][y_coordinate][2]==0 && dir_array[x_coordinate][y_coordinate][3]==0)
	{	
		switch(dir)
		{
			case 2:
			dir_array[x_coordinate][y_coordinate][0] = y;
			dir_array[x_coordinate][y_coordinate][1] = x;
			dir_array[x_coordinate][y_coordinate][2] = ny;
			dir_array[x_coordinate][y_coordinate][3] = nx;
			break;
			case 1:
			dir_array[x_coordinate][y_coordinate][1] = y;
			dir_array[x_coordinate][y_coordinate][2] = x;
			dir_array[x_coordinate][y_coordinate][3] = ny;
			dir_array[x_coordinate][y_coordinate][0] = nx;
			break;
			case -2:
			dir_array[x_coordinate][y_coordinate][2] = y;
			dir_array[x_coordinate][y_coordinate][3] = x;
			dir_array[x_coordinate][y_coordinate][0] = ny;
			dir_array[x_coordinate][y_coordinate][1] = nx;
			break;
			case -1:
			dir_array[x_coordinate][y_coordinate][3] = y;
			dir_array[x_coordinate][y_coordinate][0] = x;
			dir_array[x_coordinate][y_coordinate][1] = ny;
			dir_array[x_coordinate][y_coordinate][2] = nx;
			break;
		}
	}	
}
int is_coordinate_null(int x,int y)
{
	if(dir_array[x][y][0]==0 && dir_array[x][y][1]==0 && dir_array[x][y][2]==0 && dir_array[x][y][3]==0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
void fill_missed_array()
{
	int hx, hy, lx, ly;
	for(int i = 1 ; i<8 ; i++)
	{
		for(int j = 1 ; j<8 ; j++)
		{
			if(is_coordinate_null(i,j))
			{
				hx = i+1;
				hy = j+1;
				lx = i-1;
				ly = j-1;
				lcd_clear();
				//lcd_write_string("Null");
				//lcd_write_int_xy(0,1,i,2);
				//lcd_write_int_xy(4,1,j,2);
				//delay_millisec(2000);
				if(!(is_coordinate_null(hx,j) || is_coordinate_null(lx,j) || is_coordinate_null(i,ly) || is_coordinate_null(i,hy)))
				{
					lcd_clear();
					lcd_write_string("Filliable");
					delay_millisec(2000);
					if(dir_array[hx][j][3]==1)
					{
						dir_array[i][j][1] = 1;
					}
					else
					{
						dir_array[i][j][1] = 0;
					}
					if(dir_array[lx][j][1]==1)
					{
						dir_array[i][j][3] = 1;
					}
					else
					{
						dir_array[i][j][3] = 0;
					}
					if(dir_array[i][hy][2]==1)
					{
						dir_array[i][j][0] = 1;
					}
					else
					{
						dir_array[i][j][0] =0;
					}
					if(dir_array[i][j][ly]==1)
					{
						dir_array[i][j][2] = 1;
					}
					else
					{
						dir_array[i][j][2] = 0;
					}
				}
			}
		}
	}
}
void coordinates_tobe_reached()
{
	int stop=0;
	for(int i = 0 ; i<9 ; i++)
	{
		for(int j = 0 ; j<9 ; j++)
		{
			if(!(is_coordinate_null(i,j)))
			{
				if(dir_array[i][j][0]==1)
				{
					if(is_coordinate_null(i,j+1))
					{
						mx = i;
						my = j+1;
						stop = 1;
						break;
					}
				}
				if(dir_array[i][j][1]==1)
				{
					if(is_coordinate_null(i+1,j))
					{
						mx = i+1;
						my = j;
						stop = 1;
						break;
					}
				}
				if(dir_array[i][j][2]==1)
				{
					if(is_coordinate_null(i,j-1))
					{
						mx = i;
						my = j-1;
						stop = 1;
						break;
					}
				}
				if(dir_array[i][j][3]==1)
				{
					if(is_coordinate_null(i-1,j))
					{
						mx = i-1;
						my = j;
						stop = 1;
						break;
					}
				}	
			}	
		}
		if(stop==1)
		{
			break;
		}
	}
}


int main(void)
{
	init_devices();
	lcd_clear();
	lcd_write_string("LCD Working");
	int i, j, k;
	//array initializing to zero
	for(i=0 ; i<9 ; i++)
	{
		for(j=0 ; j<9 ; j++)
		{
			for(k=0 ; k<4 ; k++)
			{
				dir_array[i][j][k] = 0;
			}
		}
	}
	lcd_clear();
	lcd_write_string("Press Any Key");
	update_array(1,0,0,0);
	while(1)
	{	
		if(pressed_switch2() || pressed_switch0() || pressed_switch1() || pressed_switch3())
		{
			while(1)
			{
				line_track_new();	
				LED&=0b11110000;
				LED|=sensorbyte;
				right_junc_check();
				left_junc_check();
				frSensorCheck();
				irc++;
			}
		}
	}
	return 0;
}