public class CommandObserver implements Listener {

  public void register(Observable observable) {observable.add(this);}
  public void unregister(Observable observable) {observable.remove(this);}

  public void fieldChanged(Object source, String attribute) {
    System.out.println("attribute"); // this has to be implemented
  }

}