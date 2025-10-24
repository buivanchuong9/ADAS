namespace ADAS.Services;

public interface IModelService
{
    Task<InferenceResult> InferAsync(string frameB64);
}

public class InferenceResult
{
    public List<Detection> Detections { get; set; }
    public Dictionary<string, object> Stats { get; set; }
}
