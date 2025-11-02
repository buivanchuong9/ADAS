using System.Threading.Tasks;

namespace backend.Services
{
    public interface IFirebaseDataService
    {
        Task SaveDataAsync(string collection, string document, object data);
    }
}
