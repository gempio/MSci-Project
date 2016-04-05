import java.util.*;

public class CommandObject implements Observable{

  // code to maintain listeners
  private ArrayList<Listener> listeners = new ArrayList<Listener>();
  public void add(Listener listener) {listeners.add(listener);}
  public void remove(Listener listener) {listeners.remove(listener);}

  // a sample field
  private int field;
  public int getField() {return field;}
  public int setField(int value) {
    field = value;
    fire("field"); 
    return value;       
  }

  // notification code
  private void fire(String attribute) {
    for (Listener listener:listeners) {
      listener.fieldChanged(this, attribute);
    }
  }
}