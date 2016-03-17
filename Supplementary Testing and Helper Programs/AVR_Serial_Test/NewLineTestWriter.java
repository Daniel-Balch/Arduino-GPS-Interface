import java.util.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

public class NewLineTestWriter {
   public static void main(String[] args) throws FileNotFoundException {
      PrintStream output = new PrintStream(new File("Address_List.txt"));
      Map<Integer, String> binaries = new TreeMap<Integer, String>();
      for (int i = 0; i < 10000000; i++) {
         String numText = ("" + i);
         String numTextFiltered = keepCharsOnly(numText, "01");
         if (numText.equals(numTextFiltered)) {
            int index = (binaries.keySet().size());
            String leading = addLeadingZeros(numText, 7);
            binaries.put(index, leading);
         }
      }
      printNumberSwitch(output, binaries);
      output.println();
      printAddressSwitch(output, binaries);
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
   
   public static String addLeadingZeros(String sample, int desiredLength) {
      if (sample.length() < desiredLength) {
         String result = "";
         int extraZeros = (desiredLength - sample.length());
         for (int i = 0; i < extraZeros; i++) {
            result += "0";
         }
         result += sample;
         if (result.length() != desiredLength) {
            throw new IllegalStateException("addLeadingZeros method failed.");
         }   
         return result;
      } else { 
         return sample;
      }
   }            
   
   public static void printNumberSwitch(PrintStream output, Map<Integer, String> binaries) {
      output.println("\tswitch(current) {");
      for (int i = 0; i < 10; i++) {
         String current = ("" + i);
         output.println("\t\tcase '" + current + "':");
         int encoded = ((int) current.charAt(0));
         String currBin = binaries.get(encoded);
         currBin = addLeadingZeros(currBin, 8);
         String currBinLeft = reverseString((currBin.substring(0,4)));
         String currBinRight = reverseString((currBin.substring(4,8)));
         String writeCommandA = ("0b0011" + currBinLeft);
         String writeCommandB = ("0b0011" + currBinRight);
         output.println("\t\t\tbyteA = ((char) " + writeCommandA + ");");
         output.println("\t\t\tbyteB = ((char) " + writeCommandB + ");");
         output.println("\t\t\tbreak;");
      }
      output.println("\t})");
   }      
   
   public static void printAddressSwitch(PrintStream output, Map<Integer, String> binaries) {
      output.println("\tswitch (n) {");
      for (int current = 0; current < 10000000; current++) {
         if (binaries.containsKey(current)) {
            output.print("\t\tcase " + current + ":\n\t\t\t");
            String currBin = binaries.get(current);
            System.out.println("\nCurrent Number: " + current + "\nBinary: " + currBin);
            String currBinLeft = ("1" + currBin.substring(0,3));
            String currBinRight = currBin.substring(3,7);
            String addressCommandA = ("0b0001" + reverseString(currBinLeft));
            String addressCommandB = ("0b0001" + reverseString(currBinRight));
            String numberLabel = ("" + current);
            numberLabel = addLeadingZeros(numberLabel, 3);
            output.println("numLabel = \"" + numberLabel + "\";");
            output.println("\t\t\tbyteA = ((char) " + addressCommandA + ");");
            output.println("\t\t\tbyteB = ((char) " + addressCommandB + ");");
            output.println("\t\t\tbreak;");
         }
      }   
      output.print("\t}");
   }
   
   public static String reverseString(String sample) {
      String result = "";
      for (int i = sample.length(); i > 0; i--) {
         String current = sample.substring((i-1),i);
         result += current;
      }
      return result;
   }
}         

         