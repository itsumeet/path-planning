/* code taken from site url provided in the folder */


#include<stdio.h>
int main()
{
    char data[] = {'f','f','l','f','f','l','f','f','l','f','f','s'};  //Random data we want to send
    FILE *file;
    
    int i = 0;
    for(i = 0 ; i < 12; i++)
    {
	
	file = fopen("/dev/rfcomm0","w");  //Opening device file
        fprintf(file,"%c",data[i]); //Writing to the file
       
	fclose(file);        
	sleep(1);
    }
    
}
