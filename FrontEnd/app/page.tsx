import { redirect } from "next/navigation";

export default function RootPage() {
  // Redirect to overview (public landing page).
  // IntroGuard (client-side) will then bounce:
  //   - logged-in users  → /dashboard
  //   - not-logged users → /overview (allowed)
  redirect("/overview");
}
