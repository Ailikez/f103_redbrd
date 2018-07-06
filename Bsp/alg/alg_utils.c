#include "alg_utils.h"

/*
  * This is a program that convert a signed intager number to a string(length=7)
  * used various _fill char to align to length 7
  * set _fill to 0 to cancel alignment
  * example code:
  * int a = -6666, b = 233, c = 0;
    char str[7] = {'\0'};
    int main()
    {
      _itoa(a, str, ' ');
      printf(str);
      printf("\n");
      _itoa(b, str, '+');
      printf(str);
      printf("\n");
      _itoa(c, str, '-');
      printf(str);
      printf("\n");
      return 0;
    }
  */
void _itoa(int n, char *str, char _fill)
{
    /*  */
    int i, j, k;
    char s[7]={'\0'};
    signed char sign = 1;
    if (n < 0)
    {
        n = -n;
        sign = -1;
    }
    i = 0;
    do { s[i++] = n % 10 + '0'; } while ((n /= 10) > 0);
    if (sign < 0)
        s[i++] = '-';
    // reverse s[]
    if(!_fill)
    {
        for (j = i - 1, k = 0; j >= 0; j--, k++)
            str[k] = s[j];
        str[i] = '\0';
    }
    else
    {
        for(;i < 6; i++)
            s[i] = _fill;
        for (j = 5, k = 0; j >= 0; j--, k++)
            str[k] = s[j];
        str[6] = '\0';
    }
}

void _ftoa(float f, char *_str, char _fill)
{
    /*  */
    int i, j, k, n;
    char s[7]    = {'\0'};
    char s1[4]   = {'\0'};
    char str[7]  = {'\0'};
    char str1[4] = {'\0'};
    n = (int)f;
    signed char sign = 1;
    if (f < 0)
    {
        n = -n;
        sign = -1;
    }
    i = 0;
    do { s[i++] = n % 10 + '0'; } while ((n /= 10) > 0);
    if (sign < 0)
        s[i++] = '-';
    // reverse s[]
    if(!_fill)
    {
        for (j = i - 1, k = 0; j >= 0; j--, k++)
            str[k] = s[j];
        str[i] = '\0';
    }
    else
    {
        for(; i < 6; i++)
            s[i] = _fill;
        for (j = 5, k = 0; j >= 0; j--, k++)
            str[k] = s[j];
        str[6] = '\0';
    }
    n = (int)f;
    int float_part = (_abs(f) - _abs(n)) * 1000;
    i = 0;
    do { s1[i++] = float_part % 10 + '0'; } while ((float_part /= 10) > 0);

    for(j = i - 1, k = 0; j >= 0; j--, k++)
        str1[k] = s1[j];
    //str1[i] = '\0';
    for(; i < 4; i++)
    {
      if(i < 3)
        str1[i] = '0';
      else
      {
        str1[i] = '\0';
        break;
      }
    }
    for(i = 0; i < 7; i++)
    {
        if(str[i] == '\0')
        {
            _str[i] = '.';
            break;
        }
        else
            _str[i] = str[i];
    }
    for(j = 0; j < 4; j++)
    {
        if(str1[j] == '\0')
        {
            _str[++i] = '\0';
            break;
        }
        else
            _str[++i] = str1[j];
    }
}
