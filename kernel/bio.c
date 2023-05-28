// Buffer cache.
//
// The buffer cache is a linked list of buf structures holding
// cached copies of disk block contents.  Caching disk blocks
// in memory reduces the number of disk reads and also provides
// a synchronization point for disk blocks used by multiple processes.
//
// Interface:
// * To get a buffer for a particular disk block, call bread.
// * After changing buffer data, call bwrite to write it to disk.
// * When done with the buffer, call brelse.
// * Do not use the buffer after calling brelse.
// * Only one process at a time can use a buffer,
//     so do not keep them longer than necessary.


#include "types.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "riscv.h"
#include "defs.h"
#include "fs.h"
#include "buf.h"

struct {
  struct spinlock lock;
  struct spinlock bktlock[NBUCKET];
  struct buf buf[NBUF];

  // Linked list of all buffers, through prev/next.
  // Sorted by how recently the buffer was used.
  // head.next is most recent, head.prev is least.
  struct buf head[NBUCKET];
} bcache;

extern uint ticks;

int
hash(int blockno) {
  return blockno%NBUCKET;
}

void
binit(void)
{
  struct buf *b;

  initlock(&bcache.lock, "bcache");

  for (int i = 0; i<NBUCKET; i++) {
    initlock(&bcache.bktlock[i], "bcache");
  }

  // Create linked list of buffers
  for (int i = 0; i<NBUCKET; i++) {
    bcache.head[i].prev = &bcache.head[i];
    bcache.head[i].next = &bcache.head[i];
  }
  for(b = bcache.buf; b < bcache.buf+NBUF; b++){
    b->next = bcache.head[0].next;
    b->prev = &bcache.head[0];
    initsleeplock(&b->lock, "buffer");
    bcache.head[0].next->prev = b;
    bcache.head[0].next = b;
  }
}

// Look through buffer cache for block on device dev.
// If not found, allocate a buffer.
// In either case, return locked buffer.
static struct buf*
bget(uint dev, uint blockno)
{
  struct buf *b;

  int lockid = hash(blockno);

  acquire(&bcache.bktlock[lockid]);

  // Is the block already cached?
  for(b = bcache.head[lockid].next; b != &bcache.head[lockid]; b = b->next){
    if(b->dev == dev && b->blockno == blockno){
      b->refcnt++;
      release(&bcache.bktlock[lockid]);
      acquiresleep(&b->lock);
      return b;
    }
  }

  // Not cached.
  // Recycle the least recently used (LRU) unused buffer.
  struct buf *cozybuf = 0;
  uint maxtick;
  memset(&maxtick, 1, sizeof maxtick);
  for(b = bcache.head[lockid].prev; b != &bcache.head[lockid]; b = b->prev){
    if(b->refcnt == 0) {
      if (b->lastuse<maxtick) {
        maxtick = b->lastuse;
        cozybuf = b;
      }
    }
  }

  if (cozybuf == 0){
    release(&bcache.bktlock[lockid]);
    goto eviction;
  }

  // not in current bucket
  b = cozybuf;
  b->dev = dev;
  b->blockno = blockno;
  b->valid = 0;
  b->refcnt = 1;
  release(&bcache.bktlock[lockid]);
  acquiresleep(&b->lock);
  return b;

eviction:
{
  int bktid = lockid;
  int ischanged;
  struct buf *bb;

  for (int i=0; i<NBUCKET; i++) {
    if (i == lockid) continue;
    ischanged = 0;

    acquire(&bcache.bktlock[i]);

    for(b = bcache.head[i].prev; b != &bcache.head[i]; b = b->prev){
      if(b->refcnt == 0) {
        if (b->lastuse<maxtick) {
          maxtick = b->lastuse;
          cozybuf = b;
          ischanged = 1;
        }
      }
    }

    if (ischanged) {
      if (bktid != lockid)
        release(&bcache.bktlock[bktid]);
      bktid = i;
    } else {
      release(&bcache.bktlock[i]);
    }
  }

  // go bcak to lockid bucket to find the cache in case that cache has been saved 
  acquire(&bcache.bktlock[lockid]);
  for(bb = bcache.head[lockid].next; bb != &bcache.head[lockid]; bb = bb->next){
    if(bb->dev == dev && bb->blockno == blockno){
      bb->refcnt++;
      release(&bcache.bktlock[lockid]);
      if (bktid != lockid)
        release(&bcache.bktlock[bktid]);
      acquiresleep(&bb->lock);
      return bb;
    }
  }

  if (cozybuf == 0) {
    release(&bcache.bktlock[lockid]);
    if (bktid != lockid)
      release(&bcache.bktlock[bktid]);
    panic("bget: no buffers");
  }

  cozybuf->prev->next = cozybuf->next;
  cozybuf->next->prev = cozybuf->prev;
  cozybuf->next = bcache.head[lockid].next;
  cozybuf->prev = &bcache.head[lockid];
  
  bcache.head[lockid].next->prev = cozybuf;
  bcache.head[lockid].next = cozybuf;

  b = cozybuf;
  b->dev = dev;
  b->blockno = blockno;
  b->valid = 0;
  b->refcnt = 1;
  release(&bcache.bktlock[lockid]);
  release(&bcache.bktlock[bktid]);
  acquiresleep(&b->lock);
  return b;
}
}

// Return a locked buf with the contents of the indicated block.
struct buf*
bread(uint dev, uint blockno)
{
  struct buf *b;

  b = bget(dev, blockno);
  if(!b->valid) {
    virtio_disk_rw(b, 0);
    b->valid = 1;
  }
  return b;
}

// Write b's contents to disk.  Must be locked.
void
bwrite(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("bwrite");
  virtio_disk_rw(b, 1);
}

// Release a locked buffer.
// Move to the head of the most-recently-used list.
void
brelse(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("brelse");

  releasesleep(&b->lock);

  int lockid = hash(b->blockno);
  acquire(&bcache.bktlock[lockid]);
  b->refcnt--;
  if (b->refcnt == 0) {
    b->lastuse = ticks;
  }
  
  release(&bcache.bktlock[lockid]);
}

void
bpin(struct buf *b) {
  int lockid = hash(b->blockno);
  acquire(&bcache.bktlock[lockid]);
  b->refcnt++;
  release(&bcache.bktlock[lockid]);
}

void
bunpin(struct buf *b) {
  int lockid = hash(b->blockno);
  acquire(&bcache.bktlock[lockid]);
  b->refcnt--;
  release(&bcache.bktlock[lockid]);
}
