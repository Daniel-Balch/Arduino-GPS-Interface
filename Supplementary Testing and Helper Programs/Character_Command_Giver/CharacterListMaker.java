import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

public class CharacterListMaker {
   public static final String OFN = "characters.txt";
   
   public static void main(String[] args) throws FileNotFoundException {
      PrintStream output = new PrintStream(new File(OFN));
      for (int i = 0; i < 256; i++) {
         char current = ((char) i);
         output.println("" + current);
      }
   }
}         