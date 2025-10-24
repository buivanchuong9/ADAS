namespace ADAS.Services;

public interface IPerplexityService
{
    Task<string> AskAsync(string prompt);
}
