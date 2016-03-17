import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

public class BinaryListGenerator {
   public static final String OFN = "binaryList.txt";
   
   public static void main(String[] args) throws FileNotFoundException {
      PrintStream output = new PrintStream(new File(OFN));
      int counter = 0;
      for (int i = 0; i <= 11111111; i++) {
         String current = ("" + i);
         String currentFiltered = keepCharsOnly(current,"01");
         if (current.equals(currentFiltered)) {
            output.print((counter + ":" + current));
            if (i < 11111111) {
               output.println();
            }
            counter++;
         }
      }
   }            

   public static String keepCharsOnly(String sample, String keeps) {
      String result = "";
      for (int i = 0; i < sample.length(); i++) {
         String digit = sample.substring(i,(i+1));
         if (keeps.contains(digit)) {
            result += digit;
         }
      }
      return result;
   }
}