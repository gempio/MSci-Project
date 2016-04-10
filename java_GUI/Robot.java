public class Robot {
	String name;
	String location;
	int rNo;
	int energyLeft;
	boolean traveling;
	public Robot(String name, int rNo, int energyLeft, String location) {

		this.name = name;
		this.rNo = rNo;
		this.energyLeft = energyLeft;
		this.location = location;
		this.traveling = false;
	}

}