using ADAS.Services;
using Microsoft.EntityFrameworkCore;

namespace ADAS.Data;

public class AdasDbContext : DbContext
{
    public AdasDbContext(DbContextOptions<AdasDbContext> options) : base(options) { }

    public DbSet<Event> Events { get; set; }

    protected override void OnModelCreating(ModelBuilder modelBuilder)
    {
        base.OnModelCreating(modelBuilder);

        modelBuilder.Entity<Event>(entity =>
        {
            entity.HasKey(e => e.Id);
            entity.Property(e => e.EventType).IsRequired();
            entity.Property(e => e.Timestamp).IsRequired();
        });
    }
}
