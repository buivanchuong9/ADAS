using System.Text;
using System.Text.Json;

namespace ADAS.Services;

public class PerplexityService : IPerplexityService
{
    private readonly HttpClient _httpClient;
    private readonly string _apiKey;
    private readonly ILogger<PerplexityService> _logger;

    public PerplexityService(HttpClient httpClient, IConfiguration config, ILogger<PerplexityService> logger)
    {
        _httpClient = httpClient;
        _apiKey = config["PERPLEXITY_API_KEY"] ?? "";
        _logger = logger;
    }

    public async Task<string> AskAsync(string prompt)
    {
        try
        {
            var request = new
            {
                model = "pplx-7b-online",
                messages = new[] { new { role = "user", content = prompt } },
                max_tokens = 500
            };

            var content = new StringContent(
                JsonSerializer.Serialize(request),
                Encoding.UTF8,
                "application/json"
            );

            _httpClient.DefaultRequestHeaders.Authorization = new("Bearer", _apiKey);

            var response = await _httpClient.PostAsync("https://api.perplexity.ai/chat/completions", content);
            response.EnsureSuccessStatusCode();

            var json = await response.Content.ReadAsStringAsync();
            var result = JsonSerializer.Deserialize<JsonElement>(json);
            var answer = result.GetProperty("choices")[0].GetProperty("message").GetProperty("content").GetString();

            return answer ?? "Không thể lấy phản hồi";
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Perplexity API error");
            return "Lỗi: Không thể kết nối đến Perplexity API";
        }
    }
}
