using Microsoft.AspNetCore.Mvc;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using ADAS.Services;

namespace ADAS.Controllers;

[ApiController]
[Route("ws")]
public class InferenceController : ControllerBase
{
    private readonly IModelService _modelService;
    private readonly IEventService _eventService;
    private readonly ILogger<InferenceController> _logger;

    public InferenceController(IModelService modelService, IEventService eventService, ILogger<InferenceController> logger)
    {
        _modelService = modelService;
        _eventService = eventService;
        _logger = logger;
    }

    [HttpGet("infer")]
    public async Task Get()
    {
        if (HttpContext.WebSockets.IsWebSocketRequest)
        {
            using var webSocket = await HttpContext.WebSockets.AcceptWebSocketAsync();
            await HandleWebSocket(webSocket);
        }
        else
        {
            HttpContext.Response.StatusCode = StatusCodes.Status400BadRequest;
        }
    }

    private async Task HandleWebSocket(WebSocket webSocket)
    {
        var buffer = new byte[1024 * 1024 * 4]; // 4MB buffer

        try
        {
            while (webSocket.State == WebSocketState.Open)
            {
                var result = await webSocket.ReceiveAsync(new ArraySegment<byte>(buffer), CancellationToken.None);

                if (result.MessageType == WebSocketMessageType.Text)
                {
                    var json = Encoding.UTF8.GetString(buffer, 0, result.Count);
                    var request = JsonSerializer.Deserialize<FrameRequest>(json);

                    if (request?.FrameB64 != null)
                    {
                        // TODO: Call model worker for inference
                        var inference = await _modelService.InferAsync(request.FrameB64);

                        // TODO: Calculate TTC and check for collision
                        var ttc = CalculateTTC(inference.Detections);
                        if (ttc < 1.5)
                        {
                            await _eventService.LogEventAsync(new Event
                            {
                                EventType = "collision_warning",
                                Description = $"TTC: {ttc:F2}s",
                                Timestamp = DateTime.UtcNow
                            });
                        }

                        var response = JsonSerializer.Serialize(new
                        {
                            detections = inference.Detections,
                            ttc,
                            stats = inference.Stats
                        });

                        await webSocket.SendAsync(
                            Encoding.UTF8.GetBytes(response),
                            WebSocketMessageType.Text,
                            true,
                            CancellationToken.None
                        );
                    }
                }
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "WebSocket error");
        }
        finally
        {
            webSocket.Dispose();
        }
    }

    private double CalculateTTC(List<Detection> detections)
    {
        // TODO: Implement TTC calculation based on closest vehicle
        var closestCar = detections
            .Where(d => d.Cls == "car")
            .OrderBy(d => d.DistanceM)
            .FirstOrDefault();

        if (closestCar == null) return double.MaxValue;

        // Assume constant velocity of 60 km/h = 16.67 m/s
        const double velocity = 16.67;
        return closestCar.DistanceM / velocity;
    }
}

public class FrameRequest
{
    public string FrameB64 { get; set; }
}

