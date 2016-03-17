import java.util.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

public class CharacterCommandGiver {
   public static final String IFN = "characters.txt";
   public static final String BIFN = "binaryList.txt";
   
   public static void main(String[] args) throws FileNotFoundException {
      Scanner input = new Scanner(new File(IFN));
      Scanner binInput = new Scanner(new File(BIFN));
      Set<Integer> charCodes = getCharCodes(input);
      PrintStream output = new PrintStream(new File("Address_List.txt"));
      Map<Integer, String> preBinaries = getBinaries(binInput);
      Map<Integer, String> binaries = new TreeMap<Integer, String>();
      Iterator<Integer> itr = preBinaries.keySet().iterator();
      while (itr.hasNext()) {
         int current = itr.next();
         String binText = preBinaries.get(current);
         //String numText = ("" + i);
         //String numTextFiltered = keepCharsOnly(numText, "01");
         //boolean condA = numText.equals(numTextFiltered);
         //int index = (binaries.keySet().size());
         if (charCodes.contains(current)) {
            String leading = addLeadingZeros(binText, 8);
            binaries.put(current, leading);
         }
      }
      printCharacterSwitch(output, binaries);
   }
   
   public static Map<Integer, String> getBinaries(Scanner binInput) {
      Map<Integer, String> results = new TreeMap<Integer,String>();
      while (binInput.hasNextLine()) {
         String line = binInput.nextLine();
         String[] lineSegs = line.split(":");
         int number = -1;
         try {
            number = Integer.parseInt(lineSegs[0]);
         } catch (Exception e) {}
         if (number < 0) {
            throw new IllegalStateException();
         }
         results.put(number, lineSegs[1]);
      }
      return results;
   }            
   
   public static Set<Integer> getCharCodes(Scanner input) {
      Set<Integer> results = new HashSet<Integer>();
      while (input.hasNextLine()) {
         String line = input.nextLine();
         for (int i = 0; i < line.length(); i++) {
            char current = line.charAt(i);
            results.add(((int)current));
         }
      }
      return results;
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
   
   public static void printCharacterSwitch(PrintStream output, Map<Integer, String> binaries) {
      output.println("   switch(current) {");
      for (int i = 0; i < 256; i++) {
         if (binaries.containsKey(i)) {
            String current = ("" + ((char) i));
            output.println("      case '" + current + "':");
            String currBin = binaries.get(i);
            currBin = addLeadingZeros(currBin, 8);
            String currBinLeft = reverseString((currBin.substring(0,4)));
            String currBinRight = reverseString((currBin.substring(4,8)));
            String writeCommandA = ("0b0011" + currBinLeft);
            String writeCommandB = ("0b0011" + currBinRight);
            output.println("         byteA = ((char) " + writeCommandA + ");");
            output.println("         byteB = ((char) " + writeCommandB + ");");
            output.println("         break;");
         }   
      }
      output.println("   })");
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

         