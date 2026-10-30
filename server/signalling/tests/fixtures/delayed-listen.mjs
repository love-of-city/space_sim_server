import net from "node:net";

// Make premature readiness announcements fail deterministically, even on fast hosts.
const listen = net.Server.prototype.listen;
net.Server.prototype.listen = function (...args) {
  setTimeout(() => listen.apply(this, args), 300);
  return this;
};
