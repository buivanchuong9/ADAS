using System.Text;
using System.Text.Json;


using backend.Services;

namespace ADAS.Services
{
    public class ModelService : IModelService
    {
        private readonly HttpClient _httpClient;
        private readonly string _modelWorkerUrl;
        private readonly ILogger<ModelService> _logger;
        private readonly IFirebaseDataService _firebaseDataService;

        public ModelService(HttpClient httpClient, IConfiguration config, ILogger<ModelService> logger, IFirebaseDataService firebaseDataService)
        {
            _httpClient = httpClient;
            _modelWorkerUrl = config["MODEL_WORKER_URL"] ?? "http://localhost:8000";
            _logger = logger;
            _firebaseDataService = firebaseDataService;
        }

        public async Task<InferenceResult> InferAsync(string frameB64)
        {
            try
            {
                var request = new { frame_b64 = frameB64 };
                var content = new StringContent(
                    JsonSerializer.Serialize(request),
                    Encoding.UTF8,
                    "application/json"
                );

                var response = await _httpClient.PostAsync($"{_modelWorkerUrl}/infer", content);
                response.EnsureSuccessStatusCode();

                var json = await response.Content.ReadAsStringAsync();
                var result = JsonSerializer.Deserialize<InferenceResult>(json);

                // Lưu kết quả inference thực lên Firestore
                if (result != null)
                {
                    await _firebaseDataService.SaveDataAsync("inferences", Guid.NewGuid().ToString(), result);
                }

                return result ?? new InferenceResult { Detections = new(), Stats = new() };
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Model inference failed");
                return new InferenceResult { Detections = new(), Stats = new() };
            }
        }
    }
}
