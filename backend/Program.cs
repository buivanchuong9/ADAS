using ADAS.Data;
using ADAS.Services;
using Microsoft.EntityFrameworkCore;

var builder = WebApplication.CreateBuilder(args);

// Add services
builder.Services.AddControllers();
builder.Services.AddCors(options =>
{
    options.AddPolicy("AllowAll", policy =>
    {
        policy.AllowAnyOrigin().AllowAnyMethod().AllowAnyHeader();
    });
});

builder.Services.AddDbContext<AdasDbContext>(options =>
    options.UseSqlServer(builder.Configuration.GetConnectionString("DefaultConnection"))
);

builder.Services.AddScoped<IModelService, ModelService>();
builder.Services.AddScoped<IEventService, EventService>();
builder.Services.AddScoped<IPerplexityService, PerplexityService>();
builder.Services.AddHttpClient<IModelService, ModelService>();
builder.Services.AddHttpClient<IPerplexityService, PerplexityService>();

// Đăng ký FirebaseDataService
builder.Services.AddScoped<backend.Services.IFirebaseDataService, backend.Services.FirebaseDataService>();

var app = builder.Build();

// Migrate database
using (var scope = app.Services.CreateScope())
{
    var db = scope.ServiceProvider.GetRequiredService<AdasDbContext>();
    db.Database.Migrate();
}

app.UseWebSockets();
app.UseCors("AllowAll");
app.MapControllers();

app.Run("http://0.0.0.0:5000");
