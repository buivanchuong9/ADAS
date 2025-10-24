using ADAS.Data;

namespace ADAS.Services;

public class EventService : IEventService
{
    private readonly AdasDbContext _dbContext;

    public EventService(AdasDbContext dbContext)
    {
        _dbContext = dbContext;
    }

    public async Task LogEventAsync(Event @event)
    {
        _dbContext.Events.Add(@event);
        await _dbContext.SaveChangesAsync();
    }

    public async Task<List<Event>> GetEventsAsync()
    {
        return await Task.FromResult(_dbContext.Events.OrderByDescending(e => e.Timestamp).ToList());
    }
}
