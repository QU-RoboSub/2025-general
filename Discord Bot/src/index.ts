import { Hono } from 'hono';
import { IssueCommentCreatedEvent, IssuesOpenedEvent, PushEvent } from '@octokit/webhooks-types';

const app = new Hono()

app.post('/', async (c) => {
	const data = await c.req.json() as IssueCommentCreatedEvent | IssuesOpenedEvent | PushEvent;
	const headers = c.req.header();

	const githubHeaders = Object.entries(headers).reduce((acc, [key, val]) => {
		if (!key.toLowerCase().includes('github')) return acc;
		return {
			...acc,
			[key]: val,
		};
	}, {});

	const action = (data as IssueCommentCreatedEvent | IssuesOpenedEvent).action;
    const issue = (data as IssueCommentCreatedEvent | IssuesOpenedEvent).issue;
    const comment = (data as IssueCommentCreatedEvent).comment;
    const commits = (data as PushEvent).commits;
    const repository = data.repository;

    let channels: Array<keyof typeof channelMap> = [];
    const channelMap = {
		'General': '1251172177554968606/qPj4AtLGIwHNMAwVlO0yBPFO15QSd6zlLbmVhDRPHKFqeTLVNjTx1aUEVuCH_59Q8Y1Q',
		'Hull & Frame': '1251171541362933860/s7_DaYMOB_glUL1U-dKGjSIkxhKT9-RAE860gTRREbqtvt8Uz89a6c4FLd6hvWS7DwQe',
    };

    //  If the event is a new issue or comment on an issue
    if (issue) channels = (issue.labels || [ { name: 'General' } ]).map(({ name }) => name as keyof typeof channelMap);

    //  If the event is a new commit/push
    else if (commits) {}

    //  Loop through each channel and send the update
    const promises: Array<Promise<any>> = [];
    channels.forEach((name) => {
		promises.push(fetch(`https://discord.com/api/webhooks/${channelMap[name]}/github`, {
			method: 'POST',
			body: JSON.stringify(data),
			headers: {
				'accept': 'application/json',
				...githubHeaders
			},
		}));
    });

	return c.json(await Promise.all(promises));
});

export default app
