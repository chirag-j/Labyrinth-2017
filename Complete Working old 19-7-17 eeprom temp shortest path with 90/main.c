#include"sra.c"
int mpos=0;
unsigned int opt = 260;
float kp = 15;  //Proportional Gain;
float control;
float prev_error;
float act_error;
float kd = 3.1;
int flcount = 0, l = 0, lcount = 0, straight = 0 , lcountPrev = 0 , flag1 = 0, irc = 0;
int temp = 0;
char turn = 's';
int dir = 2, Coordinates_changed_by = 0 ;
int x_coordinate = 7, y_coordinate = 7 ;
char dir_array[16][16][4];
int map_array[16][16];
int mx=0, my=0;
int path_array[15][2];
int special_flag = 0, repeat=0;
int right;
int ipl = 0;
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
	delay_millisec(50);
	//delay_sec(2);
}
void turn_right()
{	
	turn = 'r';
	while(bit_is_clear(PIND, 7) || bit_is_clear(PINA, 4))
	{
		line_track_new();
		left_count();
	}
	while(bit_is_set(PIND, 7) && bit_is_set(PINA, 4))
	{
		set_pwm1a(245);
		set_pwm1b(245);
		bot_backward();
	}
	while(bit_is_set(PINA,7))
	{
		set_pwm1a(245);
		set_pwm1b(245);
		bot_spot_right();
		//left_count();
	}
	
	while((bit_is_set(PINA,6) && bit_is_set(PIND, 6)))
	{
		set_pwm1a(220);
		set_pwm1b(220);
		bot_spot_right();
		//left_count();
	}
	while(bit_is_clear(PINA,4) || bit_is_clear(PIND,7))
	{
		line_track_new();
	}
	set_pwm1a(399);
	set_pwm1b(399);
	bot_brake();
	delay_millisec(50);
}

void turn_left()
{	
	turn = 'l';
	while(bit_is_clear(PIND, 7) || bit_is_clear(PINA, 4))
	{
		line_track_new();
	}
	while(bit_is_set(PIND, 7) && bit_is_set(PINA, 4))
	{
		set_pwm1a(245);
		set_pwm1b(245);
		bot_backward();
	}
	while(bit_is_set(PINA,5))
	{
		set_pwm1a(245);
		set_pwm1b(245);
		bot_spot_left();
	}
	while(bit_is_set(PINA,6) && bit_is_set(PIND,6))
	{
		set_pwm1a(220);
		set_pwm1b(220);
		bot_spot_left();
	}
	while(bit_is_clear(PINA,4) || bit_is_clear(PIND,7))
	{
		line_track_new();
	}
	set_pwm1a(399);
	set_pwm1b(399);
	bot_brake();
	delay_millisec(50);
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

void end_zone_uturn()
{
	turn = 'u';
	while(bit_is_clear(PINA,5))
	{
		set_pwm1a(260);
		set_pwm1b(280);
		bot_spot_left();
	}
	while(bit_is_clear(PINA,6) || bit_is_clear(PIND,6));
	while(bit_is_set(PINA,5));
	while(bit_is_set(PINA,6) && bit_is_set(PIND,6))
	{
		set_pwm1a(210);
		set_pwm1b(210);
		bot_spot_left();
	}
	set_pwm1a(399);
	set_pwm1b(399);
	bot_brake();
	delay_millisec(50);
	while(bit_is_clear(PINA,4) || bit_is_clear(PIND,7))
	{
		line_track_new();
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
	
	set_pwm1a(399);
	set_pwm1b(399);
	
	bot_brake();
	//lcd_clear();
	//lcd_write_int_xy(0,0,irc,5);
	//delay_sec(20);
	
	lcd_clear();
	if(irc>2000)
	{
		// lcd_write_string("Sixty");
		if(irc>4000)
		{
			Coordinates_changed_by = 3;
		}
		else
		{
			Coordinates_changed_by = 2;
		}
	}
	else
	{
		// lcd_write_string("Thirty");
		Coordinates_changed_by = 1;
	}

	
	//delay_sec(2);
	if(irc==0)
	{
		Coordinates_changed_by = 0;
	}
	update_coordinates();
	lcountPrev = lcount;
	while(bit_is_set(PIND, 7))
	{
		line_track_new();
		left_count();
	}
	if(bit_is_clear(PINA,6) && bit_is_clear(PIND,6) && bit_is_clear(PINA,5) && bit_is_clear(PINA,7))
	{
		end_zone();
		
	}
	
	
	
	//left_count();
	set_pwm1a(399);
	set_pwm1b(399);
	bot_brake();
	lcd_clear();
	lcd_write_int_xy(0,0,lcount,1);
	//delay_sec(2);
	if(repeat<=2)
	{
		
		//End Condition
		
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
		if(straight==1 && temp>=1)
		{
			lcd_write_string_xy(0,1,"+ Detected");
			update_array('T','T','T','T');
			lcd_write_string_xy(4,0,"TTTF");
		} 
		else if(straight==0 && temp>=1)
		{
			lcd_write_string_xy(0,1,"T Detected");
			update_array('F','T','T','T');
			lcd_write_string_xy(4,0,"FTTT");
		}
		else if(straight==0 && temp==0)
		{
			lcd_write_string_xy(0,1,"L Detected");
			update_array('F','T','T','F');
			lcd_write_string_xy(4,0,"FTTF");
		}
		else if(straight==1 && temp==0)
		{
			lcd_write_string_xy(0,1,"|- Detected");
			update_array('T','T','T','F');
			lcd_write_string_xy(4,0,"TTTF");
		}
		//delay_millisec(1000);
		temp = 0;
		sense_of_directon();
	}
	// lcd_clear();
	// lcd_write_int_xy(0,0,dir,3);
	//delay_sec(2);
	
}

void frSensorCheck()
{
	if(bit_is_set(PINA,6) && bit_is_set(PIND,6))
	{
		////just confirming if its really a dead end or the bot has simply just wobbled
		
			
		//lcd_clear();
		// lcd_write_string("Uturn Detected");
		if(Coordinates_changed_by==0)
		{
			if(irc>2000)
			{
				// lcd_write_string("Sixty");
				if(irc>4000)
				{
					Coordinates_changed_by = 3;
				}
				else
				{
					Coordinates_changed_by = 2;
				}
			}
			else
			{
				// lcd_write_string("Thirty");
				Coordinates_changed_by = 1;
			}
		}
			//update_coordinates();
			//bot_brake();
			//delay_sec(2);
			
	
		
		
			
		
	}
	
	if(sensorbyte==0b0000 && bit_is_set(PINA,4) && bit_is_set(PINA,5) && bit_is_set(PINA,6) && bit_is_set(PINA,7) && bit_is_set(PIND, 6) && bit_is_set(PIND, 7))
	{
		//lcd_clear();
		//lcd_write_string("Uturn");
		bot_brake();
		update_coordinates();
		update_array('F','F','T','F');
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
		set_pwm1a(opt-30);
		set_pwm1b(opt+30);

	}
	else if(bit_is_set(PIND,6) && bit_is_clear(PINA,6))
	{
		set_pwm1a(opt+30);
		set_pwm1b(opt-30);
	}
	else if(bit_is_set(PIND,6) && bit_is_set(PINA,6))
	{
		line_track();
	}
	else if(bit_is_clear(PIND,6) && bit_is_clear(PINA,6))
	{
		set_pwm1a(opt);
		set_pwm1b(opt);
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
			// lcd_write_string("Sixty");
			if(irc>4000)
			{
				Coordinates_changed_by = 3;
			}
			else
			{
				Coordinates_changed_by = 2;
			}
		}
		else
		{
			// lcd_write_string("Thirty");
			Coordinates_changed_by = 1;
		}

		if(irc==0)
		{
			Coordinates_changed_by = 0;
		}				
	}
	if((bit_is_clear(PINA,6) || bit_is_clear(PIND,6)) && (bit_is_clear(PINA,4) || special_flag == 1))
	{
		//bot_brake();
		update_coordinates();
		//delay_sec(2);
		//lcd_clear();
		update_array('T','F','T','T');			
		// lcd_write_string("-| detected");
		//delay_millisec(500);
		irc = 0;
		special_flag = 0;       ///one time use
	}
	
	else if((bit_is_clear(PINA,4) || special_flag==1) && bit_is_set(PINA,6) && bit_is_set(PIND,6))
	{
		// lcd_clear();
		// bot_brake();
		// lcd_write_string("L detected");
		update_coordinates();
		//delay_sec(2);
		irc = 0;
		bot_brake();
		if(repeat<=2)
		{
			turn_left();						

			update_array('F','F','T','T');
			bot_brake();
			sense_of_directon();
			// lcd_clear();
			// lcd_write_int_xy(0,0,dir,3);
			// delay_millisec(2000);
			flag1 = 0;
			special_flag = 0;
		}
	}

	//special_case_check();       ///////////
	
}


void update_coordinates(void)
{	
	int value = Coordinates_changed_by;
	
	if(value==2)		//to store the middle point in 60cm shift i.e to make it continous
	{
		switch(dir)
		{
			case +1 :	x_coordinate+=1;						
						update_array('T','F','T','F');
						x_coordinate+=1;
						if(!(is_coordinate_null(x_coordinate,y_coordinate)))
						{
							repeat++;
						}
						else
						{
							repeat = 0;
						}
						break;

			case -1 :	x_coordinate-=1;						
						update_array('T','F','T','F');
						x_coordinate-=1;
						if(!(is_coordinate_null(x_coordinate,y_coordinate)))
						{
							repeat++;
						}
						else
						{
							repeat = 0;
						}
						break;
					
			case +2 :	y_coordinate+=1;						
						update_array('T','F','T','F');
						y_coordinate+=1;
						if(!(is_coordinate_null(x_coordinate,y_coordinate)))
						{
							repeat++;
						}
						else
						{
							repeat = 0;
						}
						
						break;

			case -2 :	y_coordinate-=1;						
						update_array('T','F','T','F');
						y_coordinate-=1;
						if(!(is_coordinate_null(x_coordinate,y_coordinate)))
						{
							repeat++;
						}
						else
						{
							repeat = 0;
						}
						break;	
		}
	}
	else if(value==1)
	{
		switch(dir)
		{
			case +1 : 
			x_coordinate+=value;
		    if(!(is_coordinate_null(x_coordinate,y_coordinate)))
			{
				repeat++;
			}
			else
			{
				repeat = 0;
			}
			break;
			case -1 : 
			x_coordinate-=value;
		    if(!(is_coordinate_null(x_coordinate,y_coordinate)))
			{
				repeat++;
			}
			else
			{
				repeat = 0;
			}
			break;
			case +2 : 
			y_coordinate+=value;
		    if(!(is_coordinate_null(x_coordinate,y_coordinate)))
			{
				repeat++;
			}
			else
			{
				repeat = 0;
			}
			break;
			case -2 : 
			y_coordinate-=value; 
		    if(!(is_coordinate_null(x_coordinate,y_coordinate)))
			{
				repeat++;
			}
			else
			{
				repeat = 0;
			}
			break;
		}
	}
	else if(value==3)		//to store the middle point in 60cm shift i.e to make it continous
	{
		switch(dir)
		{
			case +1 :	x_coordinate+=1;						
						update_array('T','F','T','F');
						x_coordinate+=1;
						update_array('T','F','T','F');
						x_coordinate+=1;
						if(!(is_coordinate_null(x_coordinate,y_coordinate)))
						{
							repeat++;
						}
						else
						{
							repeat = 0;
						}
						break;

			case -1 :	x_coordinate-=1;						
						update_array('T','F','T','F');
						x_coordinate-=1;
						update_array('T','F','T','F');
						x_coordinate+=1;
						if(!(is_coordinate_null(x_coordinate,y_coordinate)))
						{
							repeat++;
						}
						else
						{
							repeat = 0;
						}
						break;
					
			case +2 :	y_coordinate+=1;						
						update_array('T','F','T','F');
						y_coordinate+=1;
						update_array('T','F','T','F');
						y_coordinate+=1;
						if(!(is_coordinate_null(x_coordinate,y_coordinate)))
						{
							repeat++;
						}
						else
						{
							repeat = 0;
						}
						
						break;

			case -2 :	y_coordinate-=1;						
						update_array('T','F','T','F');
						y_coordinate-=1;
						update_array('T','F','T','F');
						y_coordinate+=1;
						if(!(is_coordinate_null(x_coordinate,y_coordinate)))
						{
							repeat++;
						}
						else
						{
							repeat = 0;
						}
						break;	
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

void update_array(char y,char x,char ny,char nx)
{
	////checking if the path is already traversed before i.e(if all 4 values is zero)
	if(is_coordinate_null(x_coordinate,y_coordinate))
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
	if(dir_array[x][y][0]=='F' && dir_array[x][y][1]=='F' && dir_array[x][y][2]=='F' && dir_array[x][y][3]=='F')
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
	for(int i = 1 ; i<15 ; i++)
	{
		for(int j = 1 ; j<15 ; j++)
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
					if(dir_array[hx][j][3]=='T')
					{
						dir_array[i][j][1] = 'T';
					}
					else
					{
						dir_array[i][j][1] = 'F';
					}
					if(dir_array[lx][j][1]=='T')
					{
						dir_array[i][j][3] = 'T';
					}
					else
					{
						dir_array[i][j][3] = 'F';
					}
					if(dir_array[i][hy][2]=='T')
					{
						dir_array[i][j][0] = 'T';
					}
					else
					{
						dir_array[i][j][0] ='F';
					}
					if(dir_array[i][j][ly]=='T')
					{
						dir_array[i][j][2] = 'T';
					}
					else
					{
						dir_array[i][j][2] = 'F';
					}
				}
			}
		}
	}
}

void coordinates_tobe_reached()
{
	int stop=0, t1, t2;
	mx = 0;
	my = 0;
	for(int i = 0 ; i<16 ; i++)
	{
		for(int j = 0 ; j<16 ; j++)
		{
			if(!(is_coordinate_null(i,j)))
			{
				if(dir_array[i][j][0]=='T')
				{
					if(is_coordinate_null(i,j+1))
					{
						if(mx==0 && my==0)
						{
							mx = i;
							my = j;
						}
						else
						{
							t1 = (((i-x_coordinate)*(i-x_coordinate))+((j-y_coordinate)*(j-y_coordinate)));   //applying distance formula
							t2 = (((mx-x_coordinate)*(mx-x_coordinate))+((my-y_coordinate)*(my-y_coordinate)));
							if(t1<t2)
							{
								mx = i;
								my = j;
							}
						}
					}
				}
				if(dir_array[i][j][1]=='T')
				{
					if(is_coordinate_null(i+1,j))
					{
						if(mx==0 && my==0)
						{
							mx = i;
							my = j;
						}
						else
						{
							t1 = (((i-x_coordinate)*(i-x_coordinate))+((j-y_coordinate)*(j-y_coordinate)));   //applying distance formula
							t2 = (((mx-x_coordinate)*(mx-x_coordinate))+((my-y_coordinate)*(my-y_coordinate)));
							
							if(t1<t2)
							{
								mx = i;
								my = j;
							}
						}
					}
				}
				if(dir_array[i][j][2]=='T')
				{
					if(is_coordinate_null(i,j-1))
					{
						if(mx==0 && my==0)
						{
							mx = i;
							my = j;
						}
						else
						{
							t1 = (((i-x_coordinate)*(i-x_coordinate))+((j-y_coordinate)*(j-y_coordinate)));   //applying distance formula
							t2 = (((mx-x_coordinate)*(mx-x_coordinate))+((my-y_coordinate)*(my-y_coordinate)));
							if(t1<t2)
							{
								mx = i;
								my = j;
							}
						}
					}
				}
				if(dir_array[i][j][3]=='T')
				{
					if(is_coordinate_null(i-1,j))
					{
						if(mx==0 && my==0)
						{
							mx = i;
							my = j;
						}
						else
						{
							t1 = (((i-x_coordinate)*(i-x_coordinate))+((j-y_coordinate)*(j-y_coordinate)));   //applying distance formula
							t2 = (((mx-x_coordinate)*(mx-x_coordinate))+((my-y_coordinate)*(my-y_coordinate)));
							if(t1<t2)
							{
								mx = i;
								my = j;
							}
						}
					}
				}	
			}	
		}
	}
}
void get_path()
{ 
	int x , y;
	y = y_coordinate;
	x = x_coordinate;
  //startingno = map_array[x_coordinate][y_coordinate];
	for(int ls = 0; ls<15; ls++)
	{
		path_array[ls][0] = 0;
		path_array[ls][1] = 0;

	}
	ipl=0;
	while(map_array[x][y]!=0)
	{
		if(map_array[x+1][y]<map_array[x][y] && dir_array[x][y][1]=='T' && dir_array[x+1][y][3]=='T' && x<15)
		{
			x = x+1;
			path_array[ipl][0]=x;
			path_array[ipl][1]=y;
			if(map_array[x+1][y]<map_array[x][y] && dir_array[x+1][y][3]=='T'  && dir_array[x][y][0]=='F'  && dir_array[x][y][1]=='T' && dir_array[x][y][2]=='F' && dir_array[x][y][3]=='T' && x<15)
			{
				x = x+1;
				path_array[ipl][0]=x;
				path_array[ipl][1]=y;
				if(map_array[x+1][y]<map_array[x][y] && dir_array[x+1][y][3]=='T'  && dir_array[x][y][0]=='F'  && dir_array[x][y][1]=='T' && dir_array[x][y][2]=='F' && dir_array[x][y][3]=='T' && x<15)
				{
					x = x+1;
					path_array[ipl][0]=x;
					path_array[ipl][1]=y;
				}
			}
		}
		else if(map_array[x-1][y]<map_array[x][y] && dir_array[x][y][3]=='T' && dir_array[x-1][y][1]=='T' && x>=1)
		{

			x = x-1;
			path_array[ipl][0]=x;
			path_array[ipl][1]=y;
			if(map_array[x-1][y]<map_array[x][y] && dir_array[x-1][y][1]=='T'  && dir_array[x][y][0]=='F'  && dir_array[x][y][1]=='T' && dir_array[x][y][2]=='F' && dir_array[x][y][3]=='T' && x>=1)
			{
				x = x-1;
				path_array[ipl][0]=x;
				path_array[ipl][1]=y;
				if(map_array[x-1][y]<map_array[x][y] && dir_array[x-1][y][1]=='T'  && dir_array[x][y][0]=='F'  && dir_array[x][y][1]=='T' && dir_array[x][y][2]=='F' && dir_array[x][y][3]=='T' && x>=1)
				{
					x = x-1;
					path_array[ipl][0]=x;
					path_array[ipl][1]=y;

				}
			}
		}

		else if(map_array[x][y+1]<map_array[x][y] && dir_array[x][y][0]=='T' && dir_array[x][y+1][2]=='T' && y<15)
		{

			y = y+1;
			path_array[ipl][0]=x;
			path_array[ipl][1]=y;
			if(map_array[x][y+1]<map_array[x][y]  && dir_array[x][y+1][2]=='T' && dir_array[x][y][0]=='T' && dir_array[x][y][1]=='F' && dir_array[x][y][2]=='T' && dir_array[x][y][3]=='F' && y<15)
			{
				y = y+1;
				path_array[ipl][0]=x;
				path_array[ipl][1]=y;
				if(map_array[x][y+1]<map_array[x][y]  && dir_array[x][y+1][2]=='T' && dir_array[x][y][0]=='T' && dir_array[x][y][1]=='F' && dir_array[x][y][2]=='T' && dir_array[x][y][3]=='F' && y<15)
				{
					y = y+1;
					path_array[ipl][0]=x;
					path_array[ipl][1]=y;

				}
			}
		}
		else if(map_array[x][y-1]<map_array[x][y] && dir_array[x][y][2]=='T' && dir_array[x][y-1][0]=='T' && y>=1)
		{

			y = y-1;
			path_array[ipl][0]=x;
			path_array[ipl][1]=y;
			if(map_array[x][y-1]<map_array[x][y] && dir_array[x][y-1][0]=='T' && dir_array[x][y][0]=='T' && dir_array[x][y][1]=='F' && dir_array[x][y][2]=='T' && dir_array[x][y][3]=='F' && y>=1)
			{
				y = y-1;
				path_array[ipl][0]=x;
				path_array[ipl][1]=y;
				if(map_array[x][y-1]<map_array[x][y] && dir_array[x][y-1][0]=='T' && dir_array[x][y][0]=='T' && dir_array[x][y][1]=='F' && dir_array[x][y][2]=='T' && dir_array[x][y][3]=='F' && y>=1)
				{
					y = y-1;
					path_array[ipl][0]=x;
					path_array[ipl][1]=y;

				}
			}
		}
		// if(is_coordinate_null(x,y))
		// {
		//   break;
		// }
		ipl++;
		if(ipl>15)
		{
			lcd_clear();
			lcd_write_string("Broke path ");
			delay_sec(2);
			break;
		}
	}
}
void build_map(int start_x , int start_y , int destn_x , int destn_y)  
{
	for (int i = 0; i < 16; i++)
	{
		for (int j = 0; j < 16; j++)
		{
			map_array[i][j] = 100;
		}
	}	
	// int tx = x, ty = y; 
	// map_array[x][y]=0;
	int fi = 0;
	map_array[destn_x][destn_y]=0;
	while(map_array[start_x][start_y]==100)
	{
		//l=x ,s=y
		
		for(int l=0; l<16; l++)
		{
			for(int s=0; s<16; s++)
			{
				if(map_array[l][s]==fi)
				{
					if(dir_array[l][s][0]=='T' && dir_array[l][s+1][2]=='T')
					{
						if(map_array[l][s+1]==100)
						{
							map_array[l][s+1]=(fi+1);
						}
					}
					if(dir_array[l][s][1]=='T' && dir_array[l+1][s][3]=='T')
					{
						if(map_array[l+1][s]==100)
						{
							map_array[l+1][s]=(fi+1);
						}
					}
					if(dir_array[l][s][2]=='T' && dir_array[l][s-1][0]=='T')
					{
						if(map_array[l][s-1]==100)
						{
							map_array[l][s-1]=(fi+1);
						}
					}
					if(dir_array[l][s][3]=='T' && dir_array[l-1][s][1]=='T')
					{
						if(map_array[l-1][s]==100)
						{
							map_array[l-1][s]=(fi+1);
						}
					}
				}
			}
		}
		fi++;
		if(fi>50)
		{
		lcd_clear();
		lcd_write_string("Broke build");
		lcd_write_int_xy(0,1,start_x,2);
		lcd_write_int_xy(3,1,start_y,2);
		lcd_write_int_xy(8,1,destn_x,2);
		lcd_write_int_xy(11,1,destn_y,2);
		delay_sec(2);
		break;
		}
	}
}

void go_to_coordinate(int next_x, int next_y)
{
	if (next_x > x_coordinate)
	{	
		checkAndCorrectDirection(1);
		while(bit_is_set(PINA,5) && bit_is_set(PINA,7))
		{
			line_track_new();

		}
		while(bit_is_set(PINA,4) && bit_is_set(PIND,7))
		{
			line_track_new();

		}
		set_pwm1b(399);
		set_pwm1a(399);
		bot_brake();
		//delay_millisec(500);
	}
	else if(next_x<x_coordinate)
	{
		checkAndCorrectDirection(-1);
		while(bit_is_set(PINA,5) && bit_is_set(PINA,7))
		{
			line_track_new();

		}
		while(bit_is_set(PINA,4) && bit_is_set(PIND,7))
		{
			line_track_new();

		}
		set_pwm1b(399);
		set_pwm1a(399);
		bot_brake();
	}

	else if (next_y>y_coordinate)
	{	
		checkAndCorrectDirection(2);
		while(bit_is_set(PINA,5) && bit_is_set(PINA,7))
		{
			line_track_new();

		}
		while(bit_is_set(PINA,4) && bit_is_set(PIND,7))
		{
			line_track_new();

		}
		set_pwm1b(399);
		set_pwm1a(399);
		bot_brake();
	}
	else if(next_y<y_coordinate)
	{
		right=0;
		checkAndCorrectDirection(-2);
		while(bit_is_set(PINA,5) && bit_is_set(PINA,7))
		{
			line_track_new();

		}
		while(bit_is_set(PINA,4) && bit_is_set(PIND,7))
		{
			line_track_new();

		}
		set_pwm1b(399);
		set_pwm1a(399);
		bot_brake();
	}	
	x_coordinate = next_x;               ///good work
	y_coordinate = next_y;
	lcd_clear();
	lcd_write_int_xy(0,1,x_coordinate,2);
	lcd_write_int_xy(5,1,y_coordinate,2);
	lcd_write_int_xy(3,0,irc,5);
	bot_brake();
	//delay_millisec(500);
}

void checkAndCorrectDirection(int req_dir)
{
	if(req_dir == 1)
	{
		switch(dir)
		{	
			
			case +2: turn_right();sense_of_directon();break;

			case -2:turn_left();sense_of_directon();break;

			case -1:
			if(dir_array[x_coordinate][y_coordinate][0]=='T')
			{
				turn_right();
				turn_right();
				dir = 1;
			}
			else
			{
				turn_right();
				dir = 1;
			}

			

		}
	}

	else if(req_dir == 2)
	{
		switch(dir)
		{	
			case +1: turn_left();sense_of_directon();break;
			case -1:turn_right();sense_of_directon();break;
			case -2:
			if(dir_array[x_coordinate][y_coordinate][3]=='T')
			{
				turn_right();
				turn_right();
				dir = 2;
			}
			else
			{
				turn_right();
				dir = 2;
			}
		}
	}

	else if(req_dir == -1)
	{
		switch(dir)
		{	
			case +2: turn_left();sense_of_directon();break;
		
			case -2:turn_right();sense_of_directon();break;

			case 1:
			if(dir_array[x_coordinate][y_coordinate][2]=='T')
			{
				turn_right();
				turn_right();
				dir = -1;
			}
			else
			{
				turn_right();
				dir = -1;
			}
			default:break;
		}
	}


	else if(req_dir == -2)
	{
		switch(dir)
		{	
			case +1:turn_right();sense_of_directon();break; 
				
			case -1: turn_left();sense_of_directon();break;
					
			case 2:
			if(dir_array[x_coordinate][y_coordinate][1]=='T')
			{
				turn_right();
				turn_right();
				dir = -2;
			}
			else
			{
				turn_right();
				dir = -2;
			}
			default:break;

		}
	}
	bot_brake();
	//delay_sec(1);
}
void map_all()
{
	while(1)
	{
		bot_brake();
		build_map(x_coordinate, y_coordinate, mx, my);
		get_path();
		
		right = 0;
		print_path();
		//delay_sec(4);
		for(int i=0; i<15; i++)
		{
			if(path_array[i][0]==0 && path_array[i][1]==0)
			{
				break;
			}
			go_to_coordinate(path_array[i][0], path_array[i][1]);

		}
		if(dir_array[x_coordinate][y_coordinate][0]=='T' && is_coordinate_null(x_coordinate,(y_coordinate+1)))
		{
			checkAndCorrectDirection(2);
		}
		else if(dir_array[x_coordinate][y_coordinate][1]=='T' && is_coordinate_null(x_coordinate+1,y_coordinate))
		{
			checkAndCorrectDirection(1);
		}
		else if(dir_array[x_coordinate][y_coordinate][2]=='T' && is_coordinate_null(x_coordinate,(y_coordinate-1)))
		{
			checkAndCorrectDirection(-2);
		}
		else if(dir_array[x_coordinate][y_coordinate][3]=='T' && is_coordinate_null(x_coordinate-1,y_coordinate))
		{
			checkAndCorrectDirection(-1);
		}
		while(bit_is_clear(PINA,4) || bit_is_clear(PIND,7))
		{
			line_track_new();
		}
		bot_brake();
		//delay_sec(2);
		irc=0;
		Coordinates_changed_by = 0;
		// if(right==1)
		// {
		// 	right_junc_check();
		// }
		// else
		// {
		// 	special_flag = 1;
		// 	left_junc_check();
		// }
		irc = 0;
		special_flag = 0;
		repeat = 0;
		while(repeat<=2)      //lsr (copied from int main) 
		{
			line_track_new();	
			LED&=0b11110000;
			LED|=sensorbyte;
			if(bit_is_clear(PINA,7))
			{
				right_junc_check();
			}
			left_junc_check();
			frSensorCheck();
			irc++;
		}
		set_pwm1a(399);
		set_pwm1b(399);
		bot_brake();
		fill_missed_array();
		mx = 0;
		my = 0;
		coordinates_tobe_reached();
		if(mx==0 && my == 0)
		{
			break;
		}
	}
}

void print_path()
{
	int a=0,b=0;
	lcd_clear();
	for(int i = 0; i<=ipl;i++)
	{
		lcd_write_int_xy(a,b, path_array[i][0],2);
		a+=2;
		lcd_write_int_xy(a,b, path_array[i][1],2);
		a+=3;
		if(a>13)
		{
			b=1;
			a=0;
		}
	}
}
void end_zone()
{
	int endx = x_coordinate, endy = y_coordinate;
	update_array('F','F','T','F');
	lcd_clear();
	set_pwm1a(399);
	set_pwm1b(399);
	bot_brake();
	flick();
	//delay_sec(1);
	build_map(7,7,endx,endy);
	int tempx = x_coordinate, tempy = y_coordinate;
	x_coordinate = 7;
	y_coordinate = 7;
	get_path();
	unsigned char eeprom_addr=0x0000;
	for(int i=0;i<15;i++)
	{
		eeprom_write_word(eeprom_addr,path_array[i][0]);
		eeprom_addr+=2;
		eeprom_write_word(eeprom_addr,path_array[i][1]);
		eeprom_addr+=2;
	}
	x_coordinate = tempx;
	y_coordinate = tempy;
	end_zone_uturn();
	sense_of_directon();
	fill_missed_array();
	coordinates_tobe_reached();
	
	lcd_clear();
	
	lcd_write_string_xy(0,0,"mx: ");
	lcd_write_int_xy(4,0,mx,2);
	lcd_write_string_xy(07,0,"my: ");
	lcd_write_int_xy(11,0,my,2);
	lcd_write_string_xy(0,1,"x: ");
	lcd_write_int_xy(4,1,x_coordinate,2);
	lcd_write_string_xy(07,1,"y: ");
	lcd_write_int_xy(11,1,y_coordinate,2);
	//delay_sec(5);
	if(mx!=0 && my!=0)
	{
		map_all();
	}
	x_coordinate = 7;
	y_coordinate = 7;
	dir = 2;
	
	
	lcd_write_string("Press Any Key");
	while(1)
	{
		if(pressed_switch2() || pressed_switch0() || pressed_switch1() || pressed_switch3())
		{
			build_map(7,7,endx,endy);
			get_path();
			unsigned char eeprom_addr=0x0000;
			for(int i=0;i<15;i++)
			{
				eeprom_write_word(eeprom_addr,path_array[i][0]);
				eeprom_addr+=2;
				eeprom_write_word(eeprom_addr,path_array[i][1]);
				eeprom_addr+=2;
			}
			lcd_clear();
			lcd_write_string("Stored In EP");
			delay_sec(1);
			right = 0;
			print_path();

			bot_brake();
			while(1)
			{				
				if(pressed_switch2() || pressed_switch0() || pressed_switch1() || pressed_switch3())
				{	
					lcd_clear();
					delay_sec(1);
					break;
				}
			}
			for(int i=0; i<15; i++)
			{
				if(path_array[i][0]==0 && path_array[i][1]==0)
				{
					break;
				}
				
				go_to_coordinate(path_array[i][0], path_array[i][1]);

			}
			while(1)
			{
				bot_stop();
				flick();
			}

		}

	}
}
int main(void)
{
	init_devices();
	lcd_clear();
	int i, j, k;
	//array initializing to false
	for(i=0 ; i<16 ; i++)
	{
		for(j=0 ; j<16; j++)
		{
			for(k=0 ; k<4 ; k++)
			{
				dir_array[i][j][k] = 'F';
			}
		}
	}
	lcd_clear();
	lcd_write_string("D3 for dry run");
	lcd_write_string_xy(0,1,"D0 for Stst path");
	update_array('T','F','F','F');
	while(1)
	{
		if(pressed_switch3())
		{
			while(1)
			{
				repeat = 0;
				while(repeat<=2)
				{
					line_track_new();	
					LED&=0b11110000;
					LED|=sensorbyte;
					if(bit_is_clear(PINA,7))
					{
						right_junc_check();
					}
					left_junc_check();
					frSensorCheck();
					irc++;
				}
				bot_brake();
				fill_missed_array();
				mx = 0;
				my = 0;
				coordinates_tobe_reached();
				// if(mx==0 && my == 0)
				// {
				// 	break;
				// }
				build_map(x_coordinate, y_coordinate, mx, my);
				get_path();
				right = 0;
				print_path();
				//delay_sec(4);
				for(int i=0; i<15; i++)
				{
					if(path_array[i][0]==0 && path_array[i][1]==0)
					{
						break;
					}
					go_to_coordinate(path_array[i][0], path_array[i][1]);
				}
				if(dir_array[x_coordinate][y_coordinate][0]=='T' && is_coordinate_null(x_coordinate,(y_coordinate+1)))
				{		
					checkAndCorrectDirection(2);
				}
				else if(dir_array[x_coordinate][y_coordinate][1]=='T' && is_coordinate_null(x_coordinate+1,y_coordinate))
				{
					checkAndCorrectDirection(1);
				}
				else if(dir_array[x_coordinate][y_coordinate][2]=='T' && is_coordinate_null(x_coordinate,(y_coordinate-1)))
				{
					checkAndCorrectDirection(-2);
				}
				else if(dir_array[x_coordinate][y_coordinate][3]=='T' && is_coordinate_null(x_coordinate-1,y_coordinate))
				{
					checkAndCorrectDirection(-1);
				}		
				while(bit_is_clear(PINA,4) || bit_is_clear(PIND,7))
				{
					line_track_new();
				}
				bot_brake();
				//delay_sec(2);
				irc=0;
				Coordinates_changed_by = 0;
				// if(right==1)
				// {
				// 	right_junc_check();
				// }
				// else
				// {
				// 	special_flag = 1;
				// 	left_junc_check();
				// }
				irc = 0;
				special_flag = 0;
				repeat = 0;
			}
		}
		if(pressed_switch0())
		{
			unsigned char eeprom_addr=0x0000;
			for(int i=0;i<15;i++)
			{
				path_array[i][0] = eeprom_read_word(eeprom_addr);
				eeprom_addr+=2;
				path_array[i][1] = eeprom_read_word(eeprom_addr);
				eeprom_addr+=2;
			}
			lcd_clear();
			lcd_write_int_xy(0,0,path_array[0][0],1);
			lcd_write_int_xy(1,0,path_array[0][1],1);
			lcd_write_int_xy(3,0,path_array[1][0],1);
			lcd_write_int_xy(4,0,path_array[1][1],1);
			lcd_write_int_xy(6,0,path_array[2][0],1);
			lcd_write_int_xy(7,0,path_array[2][1],1);
			lcd_write_int_xy(9,0,path_array[3][0],1);
			lcd_write_int_xy(10,0,path_array[3][1],1);
			lcd_write_int_xy(12,0,path_array[4][0],1);
			lcd_write_int_xy(13,0,path_array[4][1],1);
			lcd_write_int_xy(0,1,path_array[5][0],1);
			lcd_write_int_xy(1,1,path_array[5][1],1);
			while(1)
			{
				if(pressed_switch2() || pressed_switch0() || pressed_switch1() || pressed_switch3())
				{
					lcd_clear();
					delay_sec(1);
					break;
				}
			}
			for(int i=0; i<15; i++)
			{
				if(path_array[i][0]==0 && path_array[i][1]==0)
				{
					break;
				}
				go_to_coordinate(path_array[i][0], path_array[i][1]);

			}
			while(1)
			{
				bot_stop();
				flick();
				
			}
			
			
		}
	
	}
	return 0;
}