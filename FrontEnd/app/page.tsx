import { redirect } from "next/navigation";

export default function RootPage() {
  // "/" always goes to "/intro" for both logged-in and not-logged-in users.
  // IntroGuard (client-side) handles the rest:
  //   - logged-in   → can stay on /intro (rendered normally)
  //   - not-logged  → /intro is whitelisted, so also rendered normally
  redirect("/intro");
}
