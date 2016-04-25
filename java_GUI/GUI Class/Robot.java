public class Robot {
	private String name;
	private String location;
	private int rNo;
	private int energyLeft;
	private boolean traveling;
	private int cost;
	private boolean blocked;

	public Robot(String name, int rNo, int energyLeft, String location) {
		this.blocked = false;
		this.cost = 0;
		this.name = name;
		this.rNo = 0;
		this.energyLeft = energyLeft;
		this.location = location;
		this.traveling = false;
	}
	public boolean getBlocked() {
		return blocked;
	}

	public void setBlocked(boolean blocked) {
		this.blocked = blocked;
	}
	public int getRoomNumber() {
		return rNo;
	}

	public void subtractCost() {
		energyLeft -= cost;
		if(energyLeft<=0) blocked = true;
		cost = 0;
	}

	public void setLocation(int newRoom) {
		location = Integer.toString(newRoom);
		rNo = newRoom;
	}

	public void setCost(int cost) {
		this.cost = cost;
	}

	public void setTraveling(boolean travel) {
		this.traveling = travel;
	}

	public boolean isTraveling() {
		return traveling;
	}

	public String getName() {
		return name;
	}
	public String getLocation() {
		return location;
	}
	public int getRemainingEnergy() {
		return energyLeft;
	}
}