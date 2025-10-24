namespace ADAS.Services;

public interface IEventService
{
    Task LogEventAsync(Event @event);
    Task<List<Event>> GetEventsAsync();
}

public class Event
{
    public int Id { get; set; }
    public string EventType { get; set; }
    public string Description { get; set; }
    public DateTime Timestamp { get; set; }
}
