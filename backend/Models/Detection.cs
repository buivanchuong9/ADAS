namespace ADAS.Services;

public class Detection
{
    public int Id { get; set; }
    public string Cls { get; set; }
    public double Conf { get; set; }
    public List<double> Bbox { get; set; }
    public double DistanceM { get; set; }
}
