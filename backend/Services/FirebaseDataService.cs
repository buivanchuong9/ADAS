using Google.Cloud.Firestore;
using System.Threading.Tasks;

namespace backend.Services
{
    public class FirebaseDataService : IFirebaseDataService
    {
        private readonly FirestoreDb _firestoreDb;


        public FirebaseDataService()
        {
            var builder = new FirestoreDbBuilder
            {
                ProjectId = "YOUR_PROJECT_ID", // Thay YOUR_PROJECT_ID bằng project id của bạn
                Credential = Google.Apis.Auth.OAuth2.GoogleCredential.FromFile("backend/firebase-service-account.json")
            };
            _firestoreDb = builder.Build();
        }

        public async Task SaveDataAsync(string collection, string document, object data)
        {
            await _firestoreDb.Collection(collection).Document(document).SetAsync(data);
        }
    }
}
