#include <stdio.h>
#include <string.h>
#define QUIT "**quit"

int main() {
   printf("Exit command: \"%s\"\n",QUIT);
   char response[200];
   int equal = 0;
   while (equal == 0) {
      printf("\nString: ");
      scanf("%s",response);
      getchar();
      equal = compareStrings(response,QUIT);
      if (equal == 0) {
         char sum[50]; 
         checksum(sum, response);
         printf("Checksum: %s",sum);
      }
   }      
}

int checksum(char sum[], char *s) {
   int c = 0;
   while(*s) {
      c ^= *s++;
   }
   sprintf(sum,"%x",c);
   int i = 0;
   while (sum[i]) {
      sum[i] = toupper(sum[i]);
      i++;
   }   
   return 0;
}

int compareStrings(char a[], char b[]) {
   int lengthA = ((int) sizeof(a));
   int lengthB = ((int) sizeof(b));
   //printf("\nLength of A: %i\nLength of B: %i\n\n",lengthA,lengthB);
   if (lengthA == lengthB) {
      int sum = 0;
      int i;
      for (i = 0; i < lengthA; i++) {
         char x = a[i];
         char y = b[i];
         if (x == y) {
            sum++;
         }
      }
      if (sum == lengthA) {
         printf("\nExit command recognized.");
      }   
      return (sum == lengthA);
   } else {
      return 0;
   }
}               