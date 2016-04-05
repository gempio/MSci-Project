import java.util.*;

public class CommandObject implements Observable{

  // code to maintain listeners
  private ArrayList<Listener> listeners = new ArrayList<Listener>();
  public void add(Listener listener) {listeners.add(listener);}
  public void remove(Listener listener) {listeners.remove(listener);}

  // a sample field
  private String field;
  public String getField() {return field;}
  public String setField(String value) {
    field = value;
    fire(value); 
    return value;   
  }

  // notification code
  private void fire(String attribute) {
    for (Listener listener:listeners) {
      listener.fieldChanged(this, attribute);
    }
  }
}