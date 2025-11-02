using backend.Services;
using Microsoft.AspNetCore.Mvc;
using System;
using System.Threading.Tasks;

namespace backend.Controllers
{
    [ApiController]
    [Route("[controller]")]
    public class FirestoreTestController : ControllerBase
    {
        private readonly FirebaseDataService _firebaseDataService = new FirebaseDataService();

        [HttpPost("save")]
        public async Task<IActionResult> SaveEvent([FromBody] dynamic payload)
        {
            // Dữ liệu thực tế sẽ lấy từ payload hoặc từ hệ thống quét
            string eventId = payload?.EventId ?? Guid.NewGuid().ToString();
            var data = new {
                EventId = eventId,
                Type = payload?.Type ?? "detection",
                Timestamp = DateTime.UtcNow,
                Value = payload?.Value ?? 0.95
            };
            await _firebaseDataService.SaveDataAsync("events", eventId, data);
            return Ok("Saved to Firestore!");
        }
    }
}
