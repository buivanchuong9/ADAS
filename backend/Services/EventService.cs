using ADAS.Data;
using backend.Services;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace ADAS.Services
{
    public class EventService : IEventService
    {
        private readonly AdasDbContext _dbContext;
        private readonly IFirebaseDataService _firebaseDataService;

        public EventService(AdasDbContext dbContext, IFirebaseDataService firebaseDataService)
        {
            _dbContext = dbContext;
            _firebaseDataService = firebaseDataService;
        }

        public async Task LogEventAsync(Event @event)
        {
            _dbContext.Events.Add(@event);
            await _dbContext.SaveChangesAsync();
            // Đẩy dữ liệu lên Firestore
            await _firebaseDataService.SaveDataAsync("events", @event.Id.ToString(), @event);
        }

        public async Task<List<Event>> GetEventsAsync()
        {
            return await Task.FromResult(_dbContext.Events.OrderByDescending(e => e.Timestamp).ToList());
        }
    }
}
