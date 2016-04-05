public interface Observable {
  public void add(Listener listener);
  public void remove(Listener listener);
}