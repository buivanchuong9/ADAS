using FirebaseAdmin;
using Google.Apis.Auth.OAuth2;

namespace backend
{
    public static class FirebaseConfig
    {
        public static void InitFirebase()
        {
            if (FirebaseApp.DefaultInstance == null)
            {
                FirebaseApp.Create(new AppOptions()
                {
                    Credential = GoogleCredential.FromFile("backend/firebase-service-account.json")
                });
            }
        }
    }
}
