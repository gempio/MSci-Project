public class Robot {
	private String name;
	private String location;
	private int rNo;
	private int energyLeft;
	private boolean traveling;
	private int cost;

	public Robot(String name, int rNo, int energyLeft, String location) {
		this.cost = 0;
		this.name = name;
		this.rNo = 0;
		this.energyLeft = energyLeft;
		this.location = location;
		this.traveling = false;
	}

	public int getRoomNumber() {
		return rNo;
	}

	public void subtractCost() {
		energyLeft -= cost;
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
		this.traveling = traveling;
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